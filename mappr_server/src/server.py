import rospy
import rosservice

import logging

from knowledge_base_msgs.srv import *
import xml.etree.ElementTree as ET

import mappr_msgs.msg
from mappr_msgs.srv import *
from Classes.Location import Location
from Classes.Room import Room
from Classes.Location import Annotation
from Classes.Viewpoint import Viewpoint
from Classes.Positiondata import Positiondata
from Classes.Point2d import Point2d

logging.getLogger().setLevel(logging.DEBUG)


def handle_add_location(req):
    """

    :param req:
    :type req: UpdateLocationRequest
    :return:
    """

    poly = [Point2d(point.x, point.y) for point in req.location.polygon]
    annotation = Annotation(polygon=poly)

    annotation.label('room:' % req.location.label)
    room = Room(annotation=annotation)

    annotation.label('location:' % req.location.label)
    loc = Location(annotation=annotation)

    # TODO: Continue

    return UpdateLocationResponse(False)

def handle_add_fake_room(req):
    """Dirty hack to have basic functionality. Constructs a room around a received viewpoint, named after the viewpoint
    label (viewpoint label becomes Main).

    :param req: UpdateViewpointRequest
    :type req: UpdateViewpointRequest
    :rtype: bool
    :return: Whether saving was successful or not
    """
    vp_msg = req.viewpoint #type: mappr_msgs.msg.Viewpoint

    vp_position = Positiondata(
        frameid=vp_msg.header.frame_id,
        point2d=Point2d(vp_msg.pose.x, vp_msg.pose.y),
        theta=vp_msg.pose.theta
    )

    vp_obj = Viewpoint(label='Main', positiondata=vp_position)

    poly = []
    poly.append([vp_msg.pose.x + 0.5, vp_msg.pose.y + 0.5])
    poly.append([vp_msg.pose.x + 0.5, vp_msg.pose.y - 0.5])
    poly.append([vp_msg.pose.x - 0.5, vp_msg.pose.y - 0.5])
    poly.append([vp_msg.pose.x - 0.5, vp_msg.pose.y + 0.5])
    # Last point is first point again
    poly.append([vp_msg.pose.x + 0.5, vp_msg.pose.y + 0.5])

    room_annotation = Annotation(label='room:%s' % vp_msg.label, polygon=[poly], viewpoints=[vp_obj])

    fake_room = Room(name=vp_msg.label, annotation=room_annotation, numberofdoors=0)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        print(ET.tostring(fake_room.to_xml()))
        res = saving('remember %s' % ET.tostring(fake_room.to_xml()))
    except rospy.ServiceException as e:
        logging.getLogger(__name__).error("Service call failed: %s" % e)
        return UpdateViewpointResponse(False)

    if not res.success:
        logging.getLogger(__name__).error("Service call failed!")

    return UpdateViewpointResponse(res.success)


def mappr_service_server():
    rospy.init_node('mappr_service_server')

    vp_fake_add = rospy.Service('/mappr_server/add_viewpoint', mappr_msgs.srv.UpdateViewpoint, handle_add_fake_room)

    #location_add = rospy.Service('add_location', mappr_msgs.srv.UpdateLocation, handle_add_location)
    logging.getLogger(__name__).info("Server is ready")
    rospy.spin()


if __name__ == "__main__":
    if not '/KBase/query' in rosservice.get_service_list():
        logging.getLogger(__name__).critical("KBase is not running! Start KBase before running mappr")
        sys.exit(90)
    else:
        mappr_service_server()
