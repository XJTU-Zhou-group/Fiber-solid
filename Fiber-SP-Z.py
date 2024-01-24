   # -*- coding: utf-8 -*-
from abaqus import *
from abaqusConstants import *
import random
import numpy

# create model
if mdb.models.has_key("Model-1"):
    myModel = mdb.models["Model-1"]
else:
    myModel = mdb.Model(name="Model-1", modelType=STANDARD_EXPLICIT)

# initial variable
# information of base
length = 0.5
width = 0.25
height = 0.5
# information of solid fiber
fiber_length_solid = 0.25
fiber_radius_solid = 0.0125
fiber_volume_solid = 0.1
fiber_num_solid = int((fiber_volume_solid * length * width * height)/(3.14159 * fiber_length_solid * fiber_radius_solid ** 2)) + 1
print("fiber number", fiber_num_solid)
# fiber and base materials
fiber_Elastic = 75000000
fiber_Poisson_ratio = 0.25
matrix_Elastic = 1600000
matrix_Poisson_ratio = 0.35
element_size = 0.0075

# create base
myPart = myModel.Part(name="Part-base", dimensionality=THREE_D, type=DEFORMABLE_BODY)
mySketch = myModel.ConstrainedSketch(name="sketch-1", sheetSize=200)
mySketch.rectangle(point1=(0, 0), point2=(length, width))
myPart.BaseSolidExtrude(sketch=mySketch, depth=height)
# create solid fiber
myPart2 = myModel.Part(name="Part-fiber-solid", dimensionality=THREE_D, type=DEFORMABLE_BODY)
mySketch2 = myModel.ConstrainedSketch(name="sketch-2", sheetSize=200)
mySketch2.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(fiber_radius_solid, 0.0))
myPart2.BaseSolidExtrude(sketch=mySketch2, depth=fiber_length_solid)

# define interact of 
def interact_judgement(points, newpoint, fiber_radius_solid):
    c = newpoint[0]
    d = newpoint[1]
    num = 50
    sign = True
    for point in points:
        a = point[0]
        b = point[1]
        for i in range(num + 1):
            mx = c[0] + (d[0] - c[0]) * i * (1. / num)
            my = c[1] + (d[1] - c[1]) * i * (1. / num)
            mz = c[2] + (d[2] - c[2]) * i * (1. / num)
            for j in range(num + 1):
                nx = a[0] + (b[0] - a[0]) * j * (1. / num)
                ny = a[1] + (b[1] - a[1]) * j * (1. / num)
                nz = a[2] + (b[2] - a[2]) * j * (1. / num)
                distance = sqrt((mx-nx)**2+(my-ny)**2+(mz-nz)**2)
                if distance < 2 * fiber_radius_solid:
                    sign = False
                    break
            if not sign:
                break
        if not sign:
            break
    return sign

# save trans and rotate information
fiber = []
points = []
outpoints = []
# caculate the movement and rotation of fiber
# interact of judgement
fiber_real_num = 0
while fiber_real_num < fiber_num_solid:
    x = random.uniform(fiber_radius_solid, length - fiber_radius_solid)
    y = random.uniform(0, width)
    z = random.uniform(fiber_radius_solid, height - fiber_radius_solid)
    angle_y = random.uniform(0, 360)
    angle_z = 0

    z2 = z + fiber_length_solid*cos(radians(angle_y))
    x2 = x + fiber_length_solid*sin(radians(angle_y))
    y2 = y

    if fiber_radius_solid <= x2 <= length - fiber_radius_solid and fiber_radius_solid <= y2 <= width - fiber_radius_solid and fiber_radius_solid <= z2 <= height - fiber_radius_solid:
        newpoint = ((x, y, z), (x2, y2, z2))
        if len(points) == 0 or interact_judgement(points, newpoint, fiber_radius_solid):
            points.append(newpoint)
            fiber.append([x, y, z, angle_y, angle_z])
            fiber_real_num += 1
            print(str(fiber_real_num) + ' fibers have been generated!')
        else:
            pass
    elif x2 < 0 or y2 < fiber_radius_solid or z2 < 0 or x2 > length or y2 > width - fiber_radius_solid or z2 > height:
        sign = True
        for x0 in length, 0, -length:
            for y0 in width, 0, -width:
                for z0 in height, 0, -height:
                    newpoint = ((x + x0, y + y0, z + z0), (x2 + x0, y2 + y0, z2 + z0))
                    if len(points) == 0 or interact_judgement(points, newpoint, fiber_radius_solid):
                        outpoints.append(newpoint)
                    else:
                        for num in range(len(outpoints)-1, -1, -1):
                            outpoints.pop(num)
                        sign = False
                        break
                if not sign:
                    break
            if not sign:
                break
        points.extend(outpoints)
        fiber_real_num += 1
        print(str(fiber_real_num) + ' fibers have been generated!')
        for point in outpoints:
            fiber.append([point[0][0], point[0][1], point[0][2], angle_y, angle_z])
        for num in range(len(outpoints)-1, -1, -1):
            outpoints.pop(num)
    else:
        pass

# create in Abaqus
myAssembly = myModel.rootAssembly
for num in range(len(fiber)):
    x = fiber[num][0]
    y = fiber[num][1]
    z = fiber[num][2]
    angle_y = fiber[num][3]
    angle_z = fiber[num][4]
    myAssembly.Instance(name='Part-fiber-solid-{}'.format(num), part=myPart2, dependent=ON)
    myAssembly.rotate(instanceList=('Part-fiber-solid-{}'.format(num),), axisPoint=(0, 0, 0), axisDirection=(0, 1, 0),
             angle = angle_y)
    myAssembly.rotate(instanceList=('Part-fiber-solid-{}'.format(num),), axisPoint=(0, 0, 0), axisDirection=(0, 0, 1),
             angle = angle_z)
    myAssembly.translate(instanceList=('Part-fiber-solid-{}'.format(num), ), vector=(x, y, z))


# merge assembly to Part
instances = []
for ins in myAssembly.instances.values():
    instances.append(ins)
myAssembly.InstanceFromBooleanMerge(name='Part-fiber-all', instances=tuple(instances), keepIntersections=ON, originalInstances=DELETE, domain=GEOMETRY)
    
# Determine that the fiber is out of bounds
# cut fiber
p = myModel.parts['Part-fiber-all']
p.DatumCsysByThreePoints(name='Datum csys-1', coordSysType=CARTESIAN, origin=(0.0, 0.0, 0.0), line1=(1.0, 0.0, 0.0), line2=(0.0, 1.0, 0.0))
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=0.0)
p.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE, offset=length)
p.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=0.0)
p.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE, offset=height)
p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=0.0)
p.DatumPlaneByPrincipalPlane(principalPlane=XZPLANE, offset=width)
p = mdb.models['Model-1'].parts['Part-fiber-all']
session.viewports['Viewport: 1'].setValues(displayedObject=p)
p = mdb.models['Model-1'].parts['Part-fiber-all']
d = p.datums
t = p.MakeSketchTransform(sketchPlane=d[8], sketchUpEdge=d[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(50.096425, 50.0, 9.963288))
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=265.0, gridSpacing=6.62, transform=t)
g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-fiber-all']
p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
s.rectangle(point1=(-59.58, 19.86), point2=(59.58, -19.86))
p = mdb.models['Model-1'].parts['Part-fiber-all']
d2 = p.datums
p.CutExtrude(sketchPlane=d2[8], sketchUpEdge=d2[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=ON)
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
p = mdb.models['Model-1'].parts['Part-fiber-all']
d = p.datums
t = p.MakeSketchTransform(sketchPlane=d[7], sketchUpEdge=d[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(50.096425, 0.0, 9.963288))
s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=265.0, gridSpacing=6.62, transform=t)
g, v, d1, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
s1.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-fiber-all']
p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)
s1.rectangle(point1=(-59.58, 19.86), point2=(59.58, -19.86))
p = mdb.models['Model-1'].parts['Part-fiber-all']
d2 = p.datums
p.CutExtrude(sketchPlane=d2[7], sketchUpEdge=d2[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s1, flipExtrudeDirection=OFF)
s1.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
p = mdb.models['Model-1'].parts['Part-fiber-all']
d = p.datums
t = p.MakeSketchTransform(sketchPlane=d[3], sketchUpEdge=d[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(0.0, 25.0, 9.963288))
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=263.12, gridSpacing=6.57, transform=t)
g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-fiber-all']
p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
s.rectangle(point1=(-29.565, 16.425), point2=(29.565, -16.425))
p = mdb.models['Model-1'].parts['Part-fiber-all']
d2 = p.datums
p.CutExtrude(sketchPlane=d2[3], sketchUpEdge=d2[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
p = mdb.models['Model-1'].parts['Part-fiber-all']
d = p.datums
t = p.MakeSketchTransform(sketchPlane=d[4], sketchUpEdge=d[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(100.0, 25.0, 9.963288))
s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=260.85, gridSpacing=6.52, transform=t)
g, v, d1, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
s1.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-fiber-all']
p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)
s1.rectangle(point1=(-29.34, 16.3), point2=(28.4504776000977, -16.491696664917))
p = mdb.models['Model-1'].parts['Part-fiber-all']
d2 = p.datums
p.CutExtrude(sketchPlane=d2[4], sketchUpEdge=d2[2].axis3, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s1, flipExtrudeDirection=ON)
s1.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
p = mdb.models['Model-1'].parts['Part-fiber-all']
d = p.datums
t = p.MakeSketchTransform(sketchPlane=d[6], sketchUpEdge=d[2].axis2, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(50.0, 25.0, 20.0))
s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=257.7, gridSpacing=6.44, transform=t)
g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
s.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-fiber-all']
p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
s.rectangle(point1=(-56.35, 28.98), point2=(56.35, -27.37))
p = mdb.models['Model-1'].parts['Part-fiber-all']
d2 = p.datums
p.CutExtrude(sketchPlane=d2[6], sketchUpEdge=d2[2].axis2, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=ON)
s.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']
p = mdb.models['Model-1'].parts['Part-fiber-all']
d = p.datums
t = p.MakeSketchTransform(sketchPlane=d[5], sketchUpEdge=d[2].axis2, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(50.0, 25.0, 0.0))
s1 = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=254.19, gridSpacing=6.35, transform=t)
g, v, d1, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
s1.setPrimaryObject(option=SUPERIMPOSE)
p = mdb.models['Model-1'].parts['Part-fiber-all']
p.projectReferencesOntoSketch(sketch=s1, filter=COPLANAR_EDGES)
s1.rectangle(point1=(-57.15, 28.575), point2=(55.5625, -28.575))
p = mdb.models['Model-1'].parts['Part-fiber-all']
d2 = p.datums
p.CutExtrude(sketchPlane=d2[5], sketchUpEdge=d2[2].axis2, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s1, flipExtrudeDirection=OFF)
s1.unsetPrimaryObject()
del mdb.models['Model-1'].sketches['__profile__']

# assemble base
a = mdb.models['Model-1'].rootAssembly
a.DatumCsysByDefault(CARTESIAN)
p = mdb.models['Model-1'].parts['Part-base']
a.Instance(name='Part-base-1', part=p, dependent=ON)

# Materials
mdb.models['Model-1'].Material(name='Matrix')
mdb.models['Model-1'].materials['Matrix'].Elastic(table=((matrix_Elastic, matrix_Poisson_ratio), ))
mdb.models['Model-1'].Material(name='Fiber')
mdb.models['Model-1'].materials['Fiber'].Elastic(table=((fiber_Elastic, fiber_Poisson_ratio), ))
mdb.models['Model-1'].HomogeneousSolidSection(name='Section-Matrix', material='Matrix', thickness=None)
mdb.models['Model-1'].HomogeneousSolidSection(name='Section-Fiber', material='Fiber', thickness=None)
'''
# Mesh
p = mdb.models['Model-1'].parts['Part-base']
p.seedPart(size=element_size, deviationFactor=0.1, minSizeFactor=0.1)
p.generateMesh()
execfile('D:/W-sample/work/voxel-fiber.py', __main__.__dict__)

# delete Fiber
a = mdb.models['Model-1'].rootAssembly
del a.features['Part-fiber-all-1']

# Section
p = mdb.models['Model-1'].parts['Part-base']
region = p.sets['Set-Matrix']
p.SectionAssignment(region=region, sectionName='Section-Matrix', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
region = p.sets['Set-Fiber']
p.SectionAssignment(region=region, sectionName='Section-Fiber', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

# view
a = mdb.models['Model-1'].rootAssembly
a.regenerate()
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(optimizationTasks=OFF, geometricRestrictions=OFF, stopConditions=OFF)

# EasyPBC
import sys
sys.path.insert(48, r'c:/Users/dell/abaqus_plugins/abaqus_plugins/EasyPBC V.1.4')
import easypbc
easypbc.feasypbc(part='Model-1', inst='Part-base-1', meshsens=1E-07, CPU=6,
    E11=True, E22=True, E33=True, G12=True, G13=True, G23=True, onlyPBC=False,
    CTE=False, intemp=0, fntemp=100)
'''