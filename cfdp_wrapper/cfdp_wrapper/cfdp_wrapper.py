#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_message_interfaces
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Log
from rcl_interfaces.msg import ParameterDescriptor

import cfdp
import struct
from cfdp.transport.base import Transport
from cfdp.filestore import NativeFileStore
from cfdp.constants import ConditionCode, FaultHandlerAction

from cfdp_msgs.srv import CfdpCmd, CfdpXfrCmd, CfdpFileCmd
from cfe_msgs.msg import BinaryPktPayload  # Default binary packet format

DEFAULT_MAXIMUM_PACKET_LENGTH = 4096

class RosTransport(Transport):
    
    # entities provides Entity definitions and routing information
    def __init__(self, entities, maximum_packet_length=DEFAULT_MAXIMUM_PACKET_LENGTH):
        super().__init__()
        self.sent = 0
        self.entities = entities
        self.maximum_packet_length = maximum_packet_length

        # From newer transports
        # self._thread = threading.Thread(target=self._incoming_pdu_handler)
        # self._thread.kill = False
        
    def bind(self):
        print("DEBUG: RosTransport.bind")
        # Do we need to do anything here? Handled extenally for Ros Transport

    def unbind(self):
        printf("DEBUG: RosTransport.unbind")
        # Nothing to be done
        
    def request(self, data):        
        # address parameter is missing from cfdp v2.0.0 in favror of routing as parameter, but entity id is missing as parameter
        entity = self._get_entity_from_data(data)
        if not entity or not 'publisher' in entity:
            print("CFDP ERROR: Cannot send to invalid entity")
            return
        
        #print(f"CFDP Sending: len={len(data)}, Typeof {type(data)}, entity= {entity}")

        # ROS doesn't seem to allow a msg definition of a simple bytes array, so we must split it
        #  into an array of individual byte objects (byte[])
        msg_data = [i.to_bytes(1, sys.byteorder) for i in data]

        #print(f"publish msg.data=" + data.hex())
        
        entity['publisher'].publish(BinaryPktPayload(data=msg_data))
        self.sent = self.sent+1

    def _get_entity_from_data(self, data):
        pdu = cfdp.PduHeader.decode(data)
        #print(pdu, pdu.pdu_type, pdu.direction, pdu.source_entity_id, pdu.destination_entity_id, pdu.transaction_seq_number)

        if pdu.direction == 1:
            entity_id = pdu.source_entity_id
        else:
            entity_id = pdu.destination_entity_id
            
        #print(f"data has entity_id={entity_id}")
        return self.entities[entity_id]
        
        
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
        self.fileStore = self.get_parameter("filestore").get_parameter_value().string_value

        altServicesDesc = ParameterDescriptor(description="If true, append '/$entityID' to all generated services to enable testing with multiple instances")
        self.declare_parameter('altServices', False)
        useAltServices = self.get_parameter("altServices").get_parameter_value().bool_value

        pduPrefixDesc = ParameterDescriptor(description="Define topic prefix (with entityID appended) for all PDU subscriptions and publications.")
        self.declare_parameter('pduTopicPrefix', "/cfdp/pdu/entity")
        pduTopicPrefix = self.get_parameter("pduTopicPrefix").get_parameter_value().string_value

        
        cfgDescriptor = ParameterDescriptor(description='TODO: Load additional configuration parameters from file')
        self.declare_parameter("config", "src/cfe_cfdp/cfdp_wrapper/cfdp_wrapper/config/cfdp_config.yaml")
        cfg_file = self.get_parameter("config").get_parameter_value().string_value
        
        # TODO: Load entities from cfg_file (or directly from ROS parameters if possible)
        # Note: key is entity ID and should match id attribute (if not defined, it will be set automatically bleow)
        self.entities = {
            1: {
                "name": "GSW-ROS",
                "id": 1,
            },
            2: {
                "name": "FSW-ROS",
                "id": 2,
            },
            25: {
                "name": "cFE",
                "id": 25,
            }
        }
 
        local_cnt = 0
        remote_cnt = 0
        for entityid in self.entities:
            # Cleanup and sanity checks
            entity = self.entities[entityid]
            if 'id' not in entity:
                entity['id'] = entityid

            # Setup publisher/receiver details
            if entity['id'] == self.entityID:
                local_cnt += 1
            else:
                print(f" Creating publisher of " + pduTopicPrefix + str(entity['id']))
                entity['publisher'] = self.create_publisher(
                    BinaryPktPayload,
                    pduTopicPrefix + str(entity['id']),
                    10
                )
                remote_cnt += 1
                #remote_entities.append(cfdp.RemoteEntity(entity['id'], entity))


        if local_cnt!=1 or remote_cnt==0:
            print(f" local_cnt={local_cnt} and remote_cnt={remote_cnt}")
            raise ValueError("Local and Remote Entities must be defined to initialize this application")
        else:
            self.get_logger().warn(f"entityID={self.entityID}, cfg={cfg_file}entities= {self.entities}")

                    
        # Setup CFDP Entity, Transport Service, and ROS Publisher interface
        self.cfdp_ts = RosTransport(self.entities)
        self.cfdp = cfdp.CfdpEntity(
            entity_id=self.entityID,
            filestore=NativeFileStore(self.fileStore), # path is relative to script launch dir
            transport=self.cfdp_ts
            )


        # Create ROS subscription for inbound MDPUs
        self.get_logger().info('CFDP App Subscribing to ' + pduTopicPrefix + str(self.entityID))
        self._subscribe_pdu = self.create_subscription(BinaryPktPayload,
                                                        pduTopicPrefix + str(self.entityID),
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
                                                                   self.cfdp_cmd_get)
        self._trigger_cfdp_cmd_get_mkdir = self.create_service(CfdpFileCmd,
                                                             '/cfdp/cmd/mkdir' + servicePrefix,
                                                                   self.cfdp_cmd_mkdir)
        self._trigger_cfdp_cmd_get_rmdir = self.create_service(CfdpFileCmd,
                                                             '/cfdp/cmd/rmdir' + servicePrefix,
                                                                   self.cfdp_cmd_rmdir)
        self._trigger_cfdp_cmd_get_touch = self.create_service(CfdpFileCmd,
                                                             '/cfdp/cmd/touch' + servicePrefix,
                                                                   self.cfdp_cmd_touch)
        self._trigger_cfdp_cmd_get_rm = self.create_service(CfdpFileCmd,
                                                             '/cfdp/cmd/rm' + servicePrefix,
                                                                   self.cfdp_cmd_rm)


    def cfdp_cmd_ls(self, request, response):
        # WARNING: Doesn't work against cFE CFDP implementation
        self.get_logger().info("Issuing CFDP LS Command")
        self.cfdp.put(
            destination_id=request.dstid,
            transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED if request.ack else cfdp.TransmissionMode.UNACKNOWLEDGED, 
            messages_to_user=[
                cfdp.DirectoryListingRequest(
                    #remote_directory="/", local_file="/.listing.remote"
                    remote_directory=request.src, local_file=request.dst
                )])
        return response
    
    def cfdp_cmd_file(self, mode, request, response):
        # WARNING: Doesn't work against cFE CFDP implementation
        self.cfdp.put(
            destination_id=request.dstid,
            transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED if request.ack else cfdp.TransmissionMode.UNACKNOWLEDGED,
            filestore_requests=[
                cfdp.FilestoreRequest(mode, request.tgt),
                ])
        return response

    def cfdp_cmd_mkdir(self, request, response):
        self.get_logger().info(f"Issuing CFDP mkdir Command")
        return self.cfdp_cmd_file(cfdp.ActionCode.CREATE_DIRECTORY, request, response )

    def cfdp_cmd_rmdir(self, request, response):
        self.get_logger().info(f"Issuing CFDP rmdir Command")
        return self.cfdp_cmd_file(cfdp.ActionCode.REMOVE_DIRECTORY, request, response )

    def cfdp_cmd_touch(self, request, response):
        self.get_logger().info(f"Issuing CFDP touch Command")
        return self.cfdp_cmd_file(cfdp.ActionCode.CREATE_FILE, request, response )

    def cfdp_cmd_rm(self, request, response):
        self.get_logger().info(f"Issuing CFDP rm Command")
        return self.cfdp_cmd_file(cfdp.ActionCode.DELETE_FILE, request, response )

    
    def cfdp_cmd_put(self, request, response):
        dstid = request.dstid
        srcfile = request.src
        dstfile = request.dst
        self.get_logger().info(f"Issuing CFDP Put {srcfile} to {dstfile} on {dstid}")

        # Verify src file exists locally
        if not os.path.isfile(os.path.join(self.fileStore, srcfile)):
            self.get_logger().warn(f"Can't execute CFDP Put command for non-existent file {srcfile}")
            response.success=False
            return response
        
        if not self.entities[dstid]:
            self.get_logger().warn(f"Can't execute CFDP Command to unknown dstid={dstid}")
            response.success=False
            return response

        try:
            transaction_id = self.cfdp.put(
                destination_id=dstid,
                source_filename=srcfile,
                destination_filename=dstfile,
                transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED if request.ack else cfdp.TransmissionMode.UNACKNOWLEDGED, 
                fault_handler_overrides={
                    ConditionCode.POSITIVE_ACK_LIMIT_REACHED: FaultHandlerAction.ABANDON}            
            )
            response.success=True
        except BaseException as err:
            self.get_logger().error(f"Unable to execute PUT command for: {request}.  Err Type {type(err)}, Err: {err}")
            response.success=False
        
        return response
    
    def cfdp_cmd_get(self, request, response):
        self.get_logger().info("Issuing CFDP Get " + request.src + " to " + request.dst)

        if not os.path.isdir( os.path.join( self.fileStore, os.path.dirname(request.dst) ) ):
            self.get_logger().warn(f"Can't execute CFDP Get Command to invalid destination path {request.dst}.")
            response.success=False
            return response
        if not self.entities[request.dstid]:
            self.get_logger().warn(f"Can't execute CFDP Command to unknown dstid={dstid}")
            response.success=False
            return response
        
        try:
            transaction_id = self.cfdp.put(
                destination_id=request.dstid,
                transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED if request.ack else cfdp.TransmissionMode.UNACKNOWLEDGED, 

                messages_to_user=[
                    # We PUT a request to PUT a file back to us
                    cfdp.ProxyPutRequest(
                        destination_entity_id=self.entityID,
                        source_filename=request.src,
                        destination_filename=request.dst)
                ]
            )
            response.success=true
        except Exception as err:
            self.get_logger().error(f"Unable to execute GET command for: {request}.  Err Type {type(err)}, Err: {err}")
            response.success=False

        return response
    
    # Handle receipt of ROS Topic containing Binary MDPU as it's content
    def cfdp_handle_packet(self, msg):
        try:
            self.get_logger().info(f"CFDP received PDU {type(msg)} of length {len(msg.data)}")

            # Python + ROS usage of byte definitions are inconsistent, so we need to
            # explicitly rejoin an array of individual bytes into a bytes object
            bs = b''.join(msg.data)

            self.get_logger().debug(f"Handle receipt of msg.data=" + bs.hex())
        
            self.cfdp.transport.indication(bs)
        except FileNotFoundError:
            self.get_logger().warn(f"CFDP File Error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    wrapper = CFDPWrapper()
    rclpy.spin(wrapper)

    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
