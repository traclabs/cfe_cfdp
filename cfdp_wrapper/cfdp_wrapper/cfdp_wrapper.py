#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_message_interfaces
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Log
from rcl_interfaces.msg import ParameterDescriptor

import cfdp
import struct
from cfdp.transport import Transport
from cfdp.filestore import NativeFileStore
from cfdp.constants import ConditionCode, FaultHandlerAction

from cfdp_msgs.srv import CfdpCmd, CfdpXfrCmd
from cfdp_msgs.msg import CfdpPdu

class RosTransport(Transport):
    def __init__(self):
        super().__init__()
        self.sent = 0
        
    def bind(self):
        print("DEBUG: RosTransport.bind")
        # Do we need to do anything here?
        
    def request(self, data, address):
        print(f"CFDP Sending: len={len(data)}, Typeof {type(data)}, address= {type(address)}, {address}")

        # ROS doesn't seem to allow a msg definition of a simple bytes array, so we must split it
        #  into an array of individual byte objects (byte[])
        msg_data = [i.to_bytes(1, sys.byteorder) for i in data]
        address['publisher'].publish(CfdpPdu(data=msg_data))
        self.sent = self.sent+1

class CFDPWrapper(Node):

    def __init__(self):
        super().__init__('cfdp_wrapper')

        self.get_logger().warn("================================================================")
        self.get_logger().warn("CFDPWrapper")
        self.get_logger().warn("================================================================")

        # Parameters
        entityParameterDescriptor = ParameterDescriptor(description='CFDP entityID for this instance. Value must be defined in the database of CFDP entities.')
        self.declare_parameter('entityID', 2, entityParameterDescriptor)
        self.entityID = self.get_parameter("entityID").get_parameter_value().integer_value

        filestoreDescriptor = ParameterDescriptor(description='The base path (relative to script launch location) for all local file operations')
        self.declare_parameter("filestore", "cfdp/rosfsw", filestoreDescriptor)
        fileStore = self.get_parameter("filestore").get_parameter_value().string_value

        altServicesDesc = ParameterDescriptor(description="If true, append '/$entityID' to all generated services to enable testing with multiple instances")
        self.declare_parameter('altServices', False)
        useAltServices = self.get_parameter("altServices").get_parameter_value().bool_value

        cfgDescriptor = ParameterDescriptor(description='TODO: Load additional configuration parameters from file')
        self.declare_parameter("config", "src/cfe_cfdp/cfdp_wrapper/cfdp_wrapper/config/cfdp_config.yaml")
        cfg_file = self.get_parameter("config").get_parameter_value().string_value
        
        # TODO: Load entities from cfg_file (or directly from ROS parameters if possible)
        self.entities = [
            {
                "name": "GSW-ROS",
                "id": 1,
                "apid": 0x08C2
            },
            {
                "name": "FSW-ROS",
                "id": 2,
                "apid": 0x08C3
            },
            {
                "name": "cFE",
                "id": 25,
                "apid": 0x18c8
            }
        ]

        remote_entities=[]
        local_entity = None
        for entity in self.entities:
            if entity['id'] == self.entityID:
                local_entity = cfdp.LocalEntity(entity['id'], entity['apid'])
            else:
                entity['publisher'] = self.create_publisher(
                    CfdpPdu, # ByteMultiArray,  # Bytes or byteMultiArray?
                    '/cfdp/pdu/entity' + str(entity['id']),
                    10
                )
                remote_entities.append(cfdp.RemoteEntity(entity['id'], entity))


        if not local_entity or not remote_entities:
            raise ValueError("Local and Remote Entities must be defined to initialize this application")
        else:
            self.get_logger().warn(f"entityID={self.entityID}, cfg={cfg_file}entities= {self.entities}")

            

        # NOTE: Timer is not necessary, but provides useful health indicator during debugging
        self._timer_period = 10.0  # seconds
        self._timer = self.create_timer(self._timer_period, self.timer_callback)
        
        # Setup CFDP Entity, Transport Service, and ROS Publisher interface
        self.cfdp_ts = RosTransport()
        config = cfdp.Config(
            local_entity = local_entity,
            remote_entities=remote_entities,
            filestore=NativeFileStore(fileStore), # path is relative to script launch dir
            transport=self.cfdp_ts
            )
        self.cfdp = cfdp.CfdpEntity(config)

        # Create ROS subscription for inbound MDPUs
        self._subscribe_pdu = self.create_subscription(CfdpPdu,
                                                        '/cfdp/pdu/entity' + str(self.entityID),
                                                        self.cfdp_handle_packet, 10)
        
        ### Create Services for Local/User Commanding
        servicePrefix = "/entity"+str(self.entityID) if useAltServices else ""
        self._trigger_cfdp_cmd_ls_srv = self.create_service(CfdpXfrCmd,
                                                             '/cfdp/cmd/ls' + servicePrefix,
                                                                  self.cfdp_cmd_ls)
        self._trigger_cfdp_cmd_put_srv = self.create_service(CfdpXfrCmd,
                                                             '/cfdp/cmd/put' + servicePrefix,
                                                                   self.cfdp_cmd_put)
        self._trigger_cfdp_cmd_get_srv = self.create_service(CfdpXfrCmd,
                                                             '/cfdp/cmd/get' + servicePrefix,
                                                                   self.cfdp_cmd_put)


    def timer_callback(self):
        self.get_logger().warn("CFDPWrapper() -- tick")

    def cfdp_cmd_ls(self, request, response):
        # WARNING: Doesn't work against cFE CFDP implementation
        self.get_logger().info("Issuing CFDP LS Command")
        self.cfdp.put(
            destination_id=request.dstid, # 0x19, # VERIFT: WAS 1, not affecting output
            transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED,
            messages_to_user=[
                cfdp.DirectoryListingRequest(
                    #remote_directory="/", local_file="/.listing.remote"
                    remote_directory=request.src, local_file=request.dst
                )])
        return response
    
    def cfdp_cmd_put(self, request, response):
        dstid = request.dstid
        srcfile = request.src # "/test.txt"
        dstfile = request.dst # "/cf/testdst.txt"
        self.get_logger().info("Issuing CFDP Put " + srcfile + " to " + dstfile)

        try:
            transaction_id = self.cfdp.put(
                destination_id=dstid, #0x19,
                source_filename=srcfile,
                destination_filename=dstfile,
                #transmission_mode=cfdp.TransmissionMode.UNACKNOWLEDGED
                transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED,
                fault_handler_overrides={
                    ConditionCode.POSITIVE_ACK_LIMIT_REACHED: FaultHandlerAction.ABANDON}            
            )
        except Exception as err:
            self.get_logger().error(f"Unable to execute PUT command for: {request}.  Err Type {type(err)}, Err: {err}")
        
        return response
    
    def cfdp_cmd_get(self, request, response):
        self.get_logger().info("Issuing CFDP Get " + request.src + " to " + request.dst)

        try:
            transaction_id = self.cfdp.put(
                destination_id=request.dstid,
                transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED,
                messages_to_user=[
                    # We PUT a request to PUT a file back to us
                    cfdp.ProxyPutRequest(
                        destination_entity_id=self.entityID,
                        source_filename=request.src,
                        destination_filename=request.dst)
                ]
            )
        except Exception as err:
            self.get_logger().error(f"Unable to execute GET command for: {request}.  Err Type {type(err)}, Err: {err}")
        
        return response

    # Handle receipt of ROS Topic containing Binary MDPU as it's content
    def cfdp_handle_packet(self, msg):
        self.get_logger().warn(f"CFDP received PDU {type(msg)} of length {len(msg.data)}")

        # Python + ROS usage of byte definitions are inconsistent, so we need to
        # explicitly rejoin an array of individual bytes into a bytes object
        bs = b''.join(msg.data)
        
        self.cfdp.transport.indication(bs)



def main(args=None):
    rclpy.init(args=args)

    wrapper = CFDPWrapper()
    rclpy.spin(wrapper)

    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
