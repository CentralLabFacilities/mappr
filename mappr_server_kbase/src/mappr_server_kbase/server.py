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

location_array_pub = rospy.Publisher(
    '/mappr_server/current_locations',
    mappr_msgs.msg.LocationArray,
    queue_size=1,
    latch=True
)


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

    publish_current_location_vps(room)

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

    publish_current_location_vps(room)

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

    publish_current_location_vps(room)

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
        return UpdateLocationResponse(success=False, error=error)

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
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateLocationResponse(success=False, error=error)

    if not res.success:
        rospy.logerr("Service call failed!")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateLocationResponse(success=False, error=error)

    publish_current_location_vps(fake_room)

    return True, 0


def handle_add_location(req):
    """Check if location to be added already exists and if not create it.

    :param req: UpdateLocationRequest
    :return: Whether saving was successful or not.
    """

    rospy.logerr("Handling add location")
    loc_msg = req.location  # type: mappr_msgs.msg.Location

    # Construct polygon (first element is also last element)
    poly = []
    mean_x = 0
    mean_y = 0

    for point in loc_msg.polygon:
        poly.append([point.x, point.y])
        mean_x += point.x
        mean_y += point.y
    mean_x = mean_x / len(loc_msg.polygon)
    mean_y = mean_y / len(loc_msg.polygon)
    poly.append([loc_msg.polygon[0].x, loc_msg.polygon[0].y])

    # Construct label
    label = "room:%s" % loc_msg.label if not loc_msg.is_room else "location:%s" % loc_msg.label

    vp_main_position = Positiondata(
        frameid=loc_msg.header.frame_id,
        point2d=Point2d(mean_x, mean_y),
        theta=0.0
    )
    vp_main = Viewpoint(label='main', positiondata=vp_main_position)
    location_annotation = Annotation(label=label, polygon=[poly], viewpoints=[vp_main])

    if loc_msg.is_room:
        location = Room(name=loc_msg.label, annotation=location_annotation, numberofdoors=0)
        rospy.loginfo("Trying to get fetch room %s" % loc_msg.label)
        child, error = get_room(location)
    else:
        child, error = get_fake_ubiquitous_room()
        if not child:
            error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.UBIQ_ROOM_NOT_FOUND)
            return UpdateLocationResponse(success=False, error=error)

        room = Room.from_xml(child)
        location = Location(name=loc_msg.label, annotation=location_annotation, room=room)
        rospy.loginfo("Trying to get fetch location %s" % loc_msg.label)
        child, error = get_location(location)

    if child:
        rospy.loginfo("location already in DB")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.BDO_ALREADY_EXISTS)
        return UpdateLocationResponse(success=False, error=error)

    if not child and (error.code is not mappr_msgs.msg.MapprError.ROOM_NOT_FOUND or
                      error.code is not mappr_msgs.msg.MapprError.LOCATION_NOT_FOUND):
        rospy.logerr("Error while trying to read from DB! Aborting save")
        return UpdateLocationResponse(False, error)

    rospy.wait_for_service('/KBase/data')
    try:
        saving = rospy.ServiceProxy('/KBase/data', Data)
        res = saving('remember %s' % ET.tostring(location.to_xml()))
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateLocationResponse(success=False, error=error)

    if not res.success:
        rospy.logerr("Service call failed!")
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_WRITE_TO_DB)
        return UpdateLocationResponse(success=False, error=error)

    error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.NO_ERROR)
    return UpdateLocationResponse(success=True, error=error)


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

    location_add = rospy.Service(
        '/mappr_server/add_location',
        mappr_msgs.srv.UpdateLocation,
        handle_add_location
    )

    rospy.loginfo("Server is ready")

    rate = rospy.Rate(1)  # 1hz
    while not rospy.is_shutdown():
        locations, rooms, error = get_all_locations()

        rospy.loginfo("Locs %s" % len(locations))
        rospy.loginfo("Rooms %s" % len(rooms))
        for room in rooms:
            rospy.loginfo("Publishing room %s" % room.name)
            publish_current_location_vps(room)
        for loc in locations:
            rospy.loginfo("Publishing location %s" % loc.name)
            publish_current_location_vps(loc)
        publish_current_locations(locations, rooms)
        try:
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            break


def get_location(location):
    rospy.wait_for_service('/KBase/query')
    try:
        query = rospy.ServiceProxy('/KBase/query', Query)
        res = query('which location name %s' % location.name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_READ_FROM_DB)
        return None, error

    tree = ET.fromstring(res.answer.encode('utf-8'))

    error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.NO_ERROR)

    for child in tree:
        if child.tag == 'LOCATION':
            rospy.logdebug("found location %s in DB" % location.name)
            return child, 0
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.LOCATION_NOT_FOUND)
    return None, error


def get_room(room):
    rospy.wait_for_service('/KBase/query')
    try:
        query = rospy.ServiceProxy('/KBase/query', Query)
        res = query('which room name %s' % room.name)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_READ_FROM_DB)
        return None, error

    tree = ET.fromstring(res.answer.encode('utf-8'))

    error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.NO_ERROR)

    for child in tree:
        if child.tag == 'ROOM':
            rospy.logdebug("found room %s in DB" % room.name)
            return child, 0
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.ROOM_NOT_FOUND)
    return None, error


def get_all_locations():
    rospy.wait_for_service('/KBase/query')
    try:
        query = rospy.ServiceProxy('/KBase/query', Query)
        res = query('get arena')
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        error = mappr_msgs.msg.MapprError(mappr_msgs.msg.MapprError.COULD_NOT_READ_FROM_DB)
        return None, None, error

    rospy.logdebug("Retrieved all locations. Returning deserialized locations and rooms")
    arena = Arena.from_xml_local(ET.fromstring(res.answer.encode('utf-8')))

    return arena.locations, arena.rooms, 0


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


def publish_current_locations(locations, rooms):
    """Publish all locations and rooms as location array.

    :param locations: locations to publish
    :type locations: list of Location
    :param rooms: rooms to publish
    :type rooms: list of Room
    :return: None
    """
    location_msgs = []

    for loc in locations:
        loc_msg = mappr_msgs.msg.Location()
        loc_msg.label = loc.name
        loc_msg.is_room = False
        if loc.annotation.polygon:
            poly = []
            polygon = loc.annotation.polygon
            if type(polygon) == dict:  # the polygon was loaded from database
                # omit last point which is also the first one (geojson specific stuff)
                for point in polygon['coordinates'][0][:-1]:
                    poly.append(geometry_msgs.msg.Point(x=point[0], y=point[1], z=0))
            elif type(polygon) == list:  # the polygon was created "manually"
                for point in polygon[0][:-1]:  # omit last point wich is also the first one (geojson specific stuff)
                    poly.append(geometry_msgs.msg.Point(x=point[0], y=point[1], z=0))

            loc_msg.polygon = poly
        location_msgs.append(loc_msg)

    for loc in rooms:
        loc_msg = mappr_msgs.msg.Location()
        loc_msg.label = loc.name
        loc_msg.is_room = False

        if loc.annotation.polygon:
            poly = []
            polygon = loc.annotation.polygon
            if type(polygon) == dict:  # the polygon was loaded from database
                # omit last point which is also the first one (geojson specific stuff)
                for point in polygon['coordinates'][0][:-1]:
                    poly.append(geometry_msgs.msg.Point(x=point[0], y=point[1], z=0))
            elif type(polygon) == list:  # the polygon was created "manually"
                for point in polygon[0][:-1]:  # omit last point wich is also the first one (geojson specific stuff)
                    poly.append(geometry_msgs.msg.Point(x=point[0], y=point[1], z=0))

            loc_msg.polygon = poly
        location_msgs.append(loc_msg)

    location_array_pub.publish(mappr_msgs.msg.LocationArray(location_msgs))


def publish_current_location_vps(location):
    """Publish all viewpoints of the given location as viewpoint array.

    :param location: location that contains viewpoints to publish
    :type location: Room or Location
    :return: None
    """
    viewpoints = []

    for vp in location.annotation.viewpoints:
        rospy.loginfo("\t contains vp named: %s" % vp.label)
        pose_2d = geometry_msgs.msg.Pose2D(
            x=vp.positiondata.point2d.x,
            y=vp.positiondata.point2d.y,
            theta=vp.positiondata.theta
        )

        vp_msg = mappr_msgs.msg.Viewpoint(label=vp.label, parent_location_name=location.name, pose=pose_2d)
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
