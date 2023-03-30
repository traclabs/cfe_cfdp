#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosidl_runtime_py import get_message_interfaces
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Log

import cfdp
import struct
from cfdp.transport import Transport
from cfdp.filestore import NativeFileStore
from cfdp.constants import ConditionCode, FaultHandlerAction

from std_msgs.msg import ByteMultiArray

from cfdp_msgs.srv import CfdpCmd, CfdpXfrCmd


class RosTransport(Transport):
    def __init__(self, publisher):
        super().__init__()
        self.publisher = publisher;
        self.sent = 0
    def bind(self):
        print("DEBUG: RosTransport.bind")
        # Do we need to do anything here?
    def request(self, data, address):
        # TODO:  Build CCSDS Hdr then send via sbn
        print("DEBUG: Send request to ", address) # VERIFY: Is address the MID we specified in config?
        address = 0x18C8; # Force known value for initial testing
        length = len(data) +6-7  # + hdr len - min len=7
        if (address&0x1000):
            # Cmd
            secHdr = struct.pack("BB",0xCC,0xDD)  # FC and Checksum
            length += 2
        else:
            # Tlm
            secHdr = struct.pack("BBBBBB",0,0,0,0,0xAA,0xBB) # Time, TODO
            length += 6
            
        msg = (
            struct.pack(">h", address) +
            #struct.pack("BB", 0x18, 0xC8) +  # MID - TODO: If address param is correct, translate and use here
            struct.pack(">h", self.sent) + # Seq Num
            struct.pack(">h", length) + # 12+len(data)) + # Length - VERIFY
            secHdr + # Sec hdr
            data
        )

        #print("Length of list:", len(msg), " Typeof ", type(msg))
        
        self.publisher.publish(ByteMultiArray(data=[msg]))
        self.sent = self.sent+1

class CFDPWrapper(Node):

    def __init__(self):
        super().__init__('cfdp_wrapper')

        self.get_logger().warn("================================================================")
        self.get_logger().warn("CFDPWrapper")
        self.get_logger().warn("================================================================")

        # TODO: Do we need this timer? This was just part of the ros example code
        self._timer_period = 1.0  # seconds
        self._timer = self.create_timer(self._timer_period, self.timer_callback)
        
        # Setup CFDP Entity, Transport Service, and ROS Publisher interface
        self.mdpu_publisher = self.create_publisher(ByteMultiArray, '/cfdp/out', 10)
        self.cfdp_ts = RosTransport(self.mdpu_publisher)
        config = cfdp.Config(
            # TODO: Make entities configurable
            local_entity=cfdp.LocalEntity(2, 0x08C2),
            remote_entities=[
                cfdp.RemoteEntity(0x19, 0x18c8)],  # TODO/VERIFY: first param is entity_id, previously 1.
            filestore=NativeFileStore("cfdp/rosfsw"), #~/sttr/cfdp/files"), # path appears to be relative to script launch dir
            transport=self.cfdp_ts
            )
        self.cfdp = cfdp.CfdpEntity(config)

        # Create ROS subscription for inbound MDPUs
        self._subscribe_mdpu = self.create_subscription(Log, '/cfdp/in', self.cfdp_handle_packet, 10)
        
        ### Create Services for Local/User Commanding
        self._trigger_cfdp_cmd_ls_srv = self.create_service(CfdpCmd,
                                                             '/cfdp/cmd/ls',
                                                                  self.cfdp_cmd_ls)
        self._trigger_cfdp_cmd_put_srv = self.create_service(CfdpXfrCmd,
                                                             '/cfdp/cmd/put',
                                                                   self.cfdp_cmd_put)


    def timer_callback(self):
        self.get_logger().warn("CFDPWrapper() -- tick")

    def cfdp_cmd_ls(self, request, response):
        # WARNING: Doesn't work against cFE CFDP implementation
        self.get_logger().info("Issuing CFDP LS Command")
        self.cfdp.put(
            destination_id=0x19, # VERIFT: WAS 1, not affecting output
            transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED,
            messages_to_user=[
                cfdp.DirectoryListingRequest(
                    remote_directory="/", local_file="/.listing.remote")])
        return response
    
    def cfdp_cmd_put(self, request, response):
        srcfile = request.src # "/test.txt"
        dstfile = request.dst # "/cf/testdst.txt"
        self.get_logger().info("Issuing CFDP Put " + srcfile + " to " + dstfile)
        
        transaction_id = self.cfdp.put(
            destination_id=0x19,
            source_filename=srcfile,
            destination_filename=dstfile,
#            transmission_mode=cfdp.TransmissionMode.UNACKNOWLEDGED
            transmission_mode=cfdp.TransmissionMode.ACKNOWLEDGED,
            fault_handler_overrides={
                ConditionCode.POSITIVE_ACK_LIMIT_REACHED: FaultHandlerAction.ABANDON}            
        )
        
        return response

    # Handle receipt of ROS Topic containing Binary MDPU as it's content
    def cfdp_handle_packet(self, msg):
        self.get_logger().warn("DEBUG: CFDP Debug, received MDPU")
        self.cfdp.transport.indication(msg.data) # @VERIFY msg content



def main(args=None):
    rclpy.init(args=args)

    wrapper = CFDPWrapper()
    rclpy.spin(wrapper)

    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
