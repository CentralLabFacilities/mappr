import mongoengine as me
from Annotation import Annotation
import xml.etree.ElementTree as ET


class Room(me.Document):
    name = me.StringField(max_length=50, unique=True, default='')
    numberofdoors = me.IntField(default=0)
    annotation = me.EmbeddedDocumentField(Annotation)

    def to_xml(self):
        attribs = {x: self.__getattribute__(x) for x in self._fields}
        annot = attribs.pop('annotation')
        attribs.pop('id')
        attribs = {x: str(attribs[x]) for x in attribs}
        root = ET.Element('ROOM', attrib=attribs)
        root.append(annot.to_xml())

        gen = ET.SubElement(root, 'GENERATOR')
        gen.text = 'Kbase'
        time = ET.SubElement(root, 'TIMESTAMP')
        inserted = ET.SubElement(time, 'INSERTED', {'value': '1'})
        updated = ET.SubElement(time, 'UPDATED', {'value': '1'})
        return root

    @classmethod
    def from_xml(cls):
        pass