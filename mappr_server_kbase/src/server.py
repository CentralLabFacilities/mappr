#!/usr/bin/env python
import rospy
import rosservice

from knowledge_base_msgs.srv import *
import xml.etree.ElementTree as ET

import mappr_msgs.msg
import geometry_msgs.msg
from mappr_msgs.srv import *
from KnowledgeBase.Classes import *


UBIQ_ROOM = None

UBIQ_ROOM_NOT_FOUND = 97
KBASE_NOT_RUNNING = 98
BDO_DOES_NOT_EXIST = 99
BDO_ALREADY_EXISTS = 100
COULD_NOT_READ_FROM_DB = 101
COULD_NOT_WRITE_TO_DB = 102

viewpoint_array_pub = rospy.Publisher('/mappr_server/current_viewpoints', mappr_msgs.msg.ViewpointArray, queue_size=1)


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


def handle_add_viewpoint_to_fake_room(req):
    """Adds new viewpoint to ubiquitous room.

    Returns a failed state if a viewpoint with this name already exists.

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

    vp_obj = Viewpoint(label=vp_msg.label, positiondata=vp_position)

    rospy.wait_for_service('/KBase/query')
    try:
        query = rospy.ServiceProxy('/KBase/query', Query)
        res = query('which room name ubiquitous')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return UpdateViewpointResponse(success=False, error_code=COULD_NOT_READ_FROM_DB)

    found = False
    tree = ET.fromstring(res.answer.encode('utf-8'))
    for child in tree:
        if child.tag == 'ROOM':
            rospy.logdebug("Ubiquitous room found")
            found = True
            break

    if not found:
        rospy.logerr("Ubiquitous room could not be retrieved from database! Aborting save")
        return UpdateViewpointResponse(success=False, error_code=UBIQ_ROOM_NOT_FOUND)

    room = Room.from_xml(child)

    for vp in room.annotation.viewpoints:
        if vp.label == vp_msg.label:
            rospy.logerr("Could not add viewpoint since it already exists! Use the change viewpoint service to update "
                         "it")
            return UpdateViewpointResponse(success=False, error_code=BDO_ALREADY_EXISTS)

    room.annotation.viewpoints.append(vp_obj)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        res = saving('remember %s' % ET.tostring(room.to_xml()))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return UpdateViewpointResponse(success=False, error_code=COULD_NOT_WRITE_TO_DB)

    if not res.success:
        rospy.logerr("Service call failed!")
        return UpdateViewpointResponse(success=False, error_code=COULD_NOT_WRITE_TO_DB)

    publish_current_viewpoints(room)

    return UpdateViewpointResponse(success=True)


def create_ubiquitous_fake_room():
    """Check if fake room that will contain all viewpoints to be added already exists and if not create it.

    :return: Whether the room is saved in the kbase and if not with an appropriate error code
    :rtype: tuple of bool and int
    """

    rospy.wait_for_service('/KBase/query')
    try:
        query = rospy.ServiceProxy('/KBase/query', Query)
        res = query('which room name ubiquitous')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False, COULD_NOT_READ_FROM_DB

    tree = ET.fromstring(res.answer.encode('utf-8'))
    for child in tree:
        if child.tag == 'ROOM':
            rospy.loginfo("Ubiquitous room already in DB")
            return True, 0

    # Create fake 1x1 meters room
    # Last point has to be first point again
    poly = [[0.5, 0.5], [0.5, - 0.5], [- 0.5, - 0.5], [- 0.5, 0.5], [0.5, 0.5]]

    vp_position = Positiondata(
        frameid='map',
        point2d=Point2d(0.0, 0.0),
        theta=0.0
    )

    vp_obj = Viewpoint(label='main', positiondata=vp_position)
    room_annotation = Annotation(label='room:ubiquitous', polygon=[poly], viewpoints=[vp_obj])
    fake_room = Room(name='ubiquitous', annotation=room_annotation, numberofdoors=0)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        res = saving('remember %s' % ET.tostring(fake_room.to_xml()))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False, COULD_NOT_WRITE_TO_DB

    if not res.success:
        rospy.logerr("Service call failed!")
        return False, COULD_NOT_WRITE_TO_DB

    return True, 0


def mappr_service_server():
    rospy.init_node('mappr_service_server')

    rospy.loginfo("Trying to save ubiquitous fake room")
    success, error_code = create_ubiquitous_fake_room()
    if not success:
        rospy.logfatal("Saving ubiquitous fake room failed - Shutting down")
        sys.exit(error_code)

    vp_fake_add = rospy.Service(
        '/mappr_server/add_viewpoint',
        mappr_msgs.srv.UpdateViewpoint,
        handle_add_viewpoint_to_fake_room
    )

    # location_add = rospy.Service('add_location', mappr_msgs.srv.UpdateLocation, handle_add_location)
    rospy.loginfo("Server is ready")
    rospy.spin()


def publish_current_viewpoints(room):
    """Publish all viewpoints of the given room as viewpoint array.

    :param room: Room containing the viewpoints
    :type room: Room
    :return: None
    """
    viewpoints = []

    for vp in room.annotation.viewpoints:
        pose_2d = geometry_msgs.msg.Pose2D(
            x=vp.positiondata.point2d.x,
            y=vp.positiondata.point2d.y,
            theta=vp.positiondata.theta
        )

        vp_msg = mappr_msgs.msg.Viewpoint(label=vp.label, parent_location_name=room.name, pose=pose_2d)
        vp_msg.header.frame_id = vp.positiondata.frameid

        viewpoints.append(vp_msg)

    viewpoint_array_pub.publish(mappr_msgs.msg.ViewpointArray(viewpoints))


if __name__ == "__main__":
    if not '/KBase/query' in rosservice.get_service_list():
        rospy.logfatal("KBase is not running! Start KBase before running mappr")
        sys.exit(KBASE_NOT_RUNNING)
    else:
        mappr_service_server()
