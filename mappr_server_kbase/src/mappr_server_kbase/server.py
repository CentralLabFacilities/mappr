#!/usr/bin/env python
import rospy
import rosservice

import time

from knowledge_base_msgs.srv import *
import xml.etree.ElementTree as ET

import mappr_msgs.msg
import geometry_msgs.msg
from mappr_msgs.srv import *
from KnowledgeBase.Classes import *

viewpoint_array_pub = rospy.Publisher(
    '/mappr_server/current_viewpoints',
    mappr_msgs.msg.ViewpointArray,
    queue_size=1,
    latch=True
)


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


def handle_remove_viewpoint_from_fake_room(req):
    vp_msg = req.viewpoint

    child, error_code = get_fake_ubiquitous_room()

    if not child:
        rospy.logerr("arena room could not be retrieved from database! Aborting save")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.UBIQ_ROOM_NOT_FOUND)
        return UpdateViewpointResponse(success=False, error=error)

    room = Room.from_xml(child)

    found = False

    for vp in room.annotation.viewpoints:
        if vp.label == vp_msg.label:
            rospy.logdebug("found viewpoint %s in ubiq room" % vp.label)
            found = True
            break

    if not found:
        rospy.logerr("Could not remove viewpoint since it does not exists!")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.BDO_DOES_NOT_EXIST)
        return UpdateViewpointResponse(success=False, error=error)

    room.annotation.viewpoints.remove(vp)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        res = saving('remember %s' % ET.tostring(room.to_xml()))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateViewpointResponse(success=False, error=error)

    if not res.success:
        rospy.logerr("Service call failed!")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateViewpointResponse(success=False, error=error)

    publish_current_viewpoints(room)

    return UpdateViewpointResponse(success=True)


def handle_update_viewpoint_in_fake_room(req):
    vp_msg = req.viewpoint

    vp_position = Positiondata(
        frameid=vp_msg.header.frame_id,
        point2d=Point2d(vp_msg.pose.x, vp_msg.pose.y),
        theta=vp_msg.pose.theta
    )

    updated_vp = Viewpoint(label=vp_msg.label, positiondata=vp_position)

    child, error = get_fake_ubiquitous_room()

    if not child:
        rospy.logerr("arena room could not be retrieved from database! Aborting save")
        return UpdateViewpointResponse(success=False, error=error)

    room = Room.from_xml(child)

    found = False

    for vp in room.annotation.viewpoints:
        if vp.label == vp_msg.label:
            rospy.logdebug("found viewpoint %s in ubiq room" % vp.label)
            found = True
            break

    if not found:
        rospy.logerr("Could not remove viewpoint since it does not exists!")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.BDO_DOES_NOT_EXIST)
        return UpdateViewpointResponse(success=False, error=error)

    room.annotation.viewpoints.remove(vp)
    room.annotation.viewpoints.append(updated_vp)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        res = saving('remember %s' % ET.tostring(room.to_xml()))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateViewpointResponse(success=False, error=error)

    if not res.success:
        rospy.logerr("Service call failed!")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateViewpointResponse(success=False, error=error)

    publish_current_viewpoints(room)

    return UpdateViewpointResponse(success=True)


def handle_add_viewpoint_to_fake_room(req):
    """Adds new viewpoint to arena room.

    Returns a failed state if a viewpoint with this name already exists.

    :param req: UpdateViewpointRequest
    :type req: UpdateViewpointRequest
    :rtype: bool
    :return: Whether saving was successful or not
    """
    vp_msg = req.viewpoint

    vp_position = Positiondata(
        frameid=vp_msg.header.frame_id,
        point2d=Point2d(vp_msg.pose.x, vp_msg.pose.y),
        theta=vp_msg.pose.theta
    )

    vp_obj = Viewpoint(label=vp_msg.label, positiondata=vp_position)

    child, error = get_fake_ubiquitous_room()

    if not child:
        rospy.logerr("arena room could not be retrieved from database! Aborting save")
        return UpdateViewpointResponse(success=False, error=error)

    room = Room.from_xml(child)

# Temporary workaround: re-saving existing vps is allowed since updating is not implemented in the plugin yet.
# TODO: Remove after updating viewpoints is implemented
    
    for vp in room.annotation.viewpoints:
        if vp.label == vp_msg.label:
            room.annotation.viewpoints.remove(vp)
#            rospy.logerr("Could not add viewpoint since it already exists! Use the change viewpoint service to update "
#                         "it")
#            error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.BDO_ALREADY_EXISTS)
#            return UpdateViewpointResponse(success=False, error=error)

    room.annotation.viewpoints.append(vp_obj)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        res = saving('remember %s' % ET.tostring(room.to_xml()))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateViewpointResponse(success=False, error=error)

    if not res.success:
        rospy.logerr("Service call failed!")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateViewpointResponse(success=False, error=error)

    publish_current_viewpoints(room)

    return UpdateViewpointResponse(success=True)


def create_ubiquitous_fake_room():
    """Check if fake room that will contain all viewpoints to be added already exists and if not create it.

    :return: Whether the room is saved in the kbase and if not with an appropriate error code
    :rtype: tuple of bool and int
    """

    rospy.loginfo("Trying to get arena fake room")
    child, error = get_fake_ubiquitous_room()

    if child:
        rospy.loginfo("arena room already in DB")
        return True, 0

    if not child and error.code is not mappr_msgs.msg.MapprError.UBIQ_ROOM_NOT_FOUND:
        rospy.logerr("Error while trying to read from DB! Aborting save")
        return None, error.code

    # Create fake 1x1 meters room
    # Last point has to be first point again
    poly = [[0.5, 0.5], [0.5, - 0.5], [- 0.5, - 0.5], [- 0.5, 0.5], [0.5, 0.5]]

    vp_position = Positiondata(
        frameid='map',
        point2d=Point2d(0.0, 0.0),
        theta=0.0
    )

    vp_obj = Viewpoint(label='main', positiondata=vp_position)
    room_annotation = Annotation(label='room:arena', polygon=[poly], viewpoints=[vp_obj])
    fake_room = Room(name='arena', annotation=room_annotation, numberofdoors=0)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        res = saving('remember %s' % ET.tostring(fake_room.to_xml()))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False, mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB

    if not res.success:
        rospy.logerr("Service call failed!")
        return False, mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB

    publish_current_viewpoints(fake_room)

    return True, 0


def mappr_service_server():

    rospy.loginfo("Trying to save arena fake room")
    success, error_code = create_ubiquitous_fake_room()
    if not success:
        rospy.logfatal("Saving arena fake room failed - Shutting down")
        sys.exit(error_code)

    vp_fake_add = rospy.Service(
        '/mappr_server/add_viewpoint',
        mappr_msgs.srv.UpdateViewpoint,
        handle_add_viewpoint_to_fake_room
    )

    vp_fake_update = rospy.Service(
        '/mappr_server/update_viewpoint',
        mappr_msgs.srv.UpdateViewpoint,
        handle_update_viewpoint_in_fake_room
    )

    vp_fake_remove = rospy.Service(
        '/mappr_server/remove_viewpoint',
        mappr_msgs.srv.UpdateViewpoint,
        handle_remove_viewpoint_from_fake_room
    )

    # location_add = rospy.Service('add_location', mappr_msgs.srv.UpdateLocation, handle_add_location)
    rospy.loginfo("Server is ready")

    rate = rospy.Rate(0.2)  # 0.2hz
    while not rospy.is_shutdown():
        child, error_code = get_fake_ubiquitous_room()

        if child is None:
            rospy.logerr("arena room could not be retrieved from database! Can not publish viewpoints")

        room = Room.from_xml(child)
        publish_current_viewpoints(room)
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break


def get_fake_ubiquitous_room():
    rospy.wait_for_service('/KBase/query')
    try:
        query = rospy.ServiceProxy('/KBase/query', Query)
        res = query('which room name arena')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_READ_FROM_DB)
        return None, error

    tree = ET.fromstring(res.answer.encode('utf-8'))

    error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.NO_ERROR)

    for child in tree:
        if child.tag == 'ROOM':
            rospy.logdebug("arena room found in DB")
            return child, 0
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.UBIQ_ROOM_NOT_FOUND)
    return None, error


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
    rospy.init_node('mappr_service_server')

    tries = 0
    kbase_found = False

    while tries < 3:
        if '/KBase/query' in rosservice.get_service_list():
            kbase_found = True
            break
        tries += 1
        time.sleep(1)

    if not kbase_found:
        rospy.logfatal("KBase is not running! Start KBase before running mappr")
        sys.exit(mappr_msgs.msg.MapprError.KBASE_NOT_RUNNING)
    else:
        mappr_service_server()
