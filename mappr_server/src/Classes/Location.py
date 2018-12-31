import mongoengine as me
from Annotation import Annotation
from Room import Room
from Exceptions import NoSuchRoomException
import xml.etree.ElementTree as ET


class Location(me.Document):
    name = me.StringField(max_length=50, unique=True, default='')
    room = me.ReferenceField(Room)
    isbeacon = me.BooleanField(default=False)
    isplacement = me.BooleanField(default=False)
    ishidden = me.BooleanField(default=False)
    annotation = me.EmbeddedDocumentField(Annotation)

    def to_xml(self):
        attribs = {x: self.__getattribute__(x) for x in self._fields}
        annot = attribs.pop('annotation')
        attribs.pop('id')
        attribs['room'] = attribs['room'].name #todo: room as reference?
        attribs = {x: str(attribs[x]) for x in attribs}
        root = ET.Element('LOCATION', attrib=attribs)
        root.append(annot.to_xml())

        gen = ET.SubElement(root, 'GENERATOR')
        gen.text = 'Kbase'
        time = ET.SubElement(root, 'TIMESTAMP')
        inserted = ET.SubElement(time, 'INSERTED', {'value': '1'})
        updated = ET.SubElement(time, 'UPDATED', {'value': '1'})
        return root

    @classmethod
    def from_xml(cls, xml_tree):
        loc = Location()
        loc.name = xml_tree.get('name')
        loc.isbeacon = xml_tree.get('isbeacon') == 'true'
        loc.isplacement = xml_tree.get('isplacement') == 'true'

        for potential_annot in xml_tree.getchildren():
            if potential_annot.tag.lower() == Annotation.get_tag().lower():
                loc.annotation = Annotation.from_xml(potential_annot)

        room = xml_tree.get('room')
        room_doc = Room.objects(name=room).get()
        if not type(room_doc) == Room:
            raise NoSuchRoomException()
        loc.room = room_doc

        return loc

    @classmethod
    def get_tag(cls):
        return 'LOCATION'
