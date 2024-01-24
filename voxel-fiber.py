# encoding=utf-8

from abaqus import *
from abaqusConstants import *
import numpy as np

model = mdb.models["Model-1"]

partGear = model.parts["Part-fiber-all"]
partBase = model.parts["Part-base"]

elements = partBase.elements
nodes = partBase.nodes

labels, otherLabels = [], []
cells = partGear.cells

for element in elements:

    nodeIndex = element.connectivity
    center = np.array([0.0, 0.0, 0.0])

    for index in nodeIndex:
        center += nodes[index].coordinates
    center /= 8

    findCell = cells.findAt((center,), printWarning=False)

    if len(findCell):
        labels.append(element.label)
    else:
        otherLabels.append(element.label)

partBase.Set(elements=elements.sequenceFromLabels(labels=labels), name="Set-Fiber")
partBase.Set(elements=elements.sequenceFromLabels(labels=otherLabels), name="Set-Matrix")
