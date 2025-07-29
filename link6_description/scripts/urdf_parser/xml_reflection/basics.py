from xml.dom import minidom
from xml.etree import ElementTree as ET


def xml_string(rootXml, addHeader=True):
    # From: https://stackoverflow.com/a/1206856/170413
    # TODO(eacousineau): This does not preserve attribute order. Fix it.
    dom = minidom.parseString(ET.tostring(rootXml))
    xml_string = ""
    lines = dom.toprettyxml(indent="  ").split("\n")
    if lines and lines[0].startswith("<?xml") and not addHeader:
        del lines[0]
    # N.B. Minidom injects some pure-whitespace lines. Remove them.
    return "\n".join(filter(lambda line: line.strip(), lines))


def dict_sub(obj, keys):
    return dict((key, obj[key]) for key in keys)


def node_add(doc, sub):
    if sub is None:
        return None
    if type(sub) == str:
        return ET.SubElement(doc, sub)
    elif isinstance(sub, ET.Element):
        doc.append(sub)  # This screws up the rest of the tree for prettyprint
        return sub
    else:
        raise Exception("Invalid sub value")


def pfloat(x):
    return str(x).rstrip(".")


def xml_children(node):
    return list(node)


def isstring(obj):
    return isinstance(obj, str)


class SelectiveReflection(object):
    def get_refl_vars(self):
        return list(vars(self).keys())
