from yade import pack, ymport, timing, plot

O.materials.append(FrictMat(density=7.65e3,young=1.95e11,poisson=0.3,frictionAngle=0.1,label='316L'))

workBenchIds=O.bodies.append(ymport.stl('box.stl',wire=True))
rollerIds=O.bodies.append(ymport.stl('roller-Layer3.stl',wire=True))
firstLayerIds=O.bodies.append(ymport.stl('contour_layer2.stl',wire=True))

sp=pack.SpherePack()
sp.makeCloud((0,0,90),(1500,200,340),
psdSizes=[2,5,6.18,7.5,9.08,10.5,12.63,15,17.73,20.52,22.1,24.6,27.3,30,32.5,35],
psdCumm=[0,0.0461,0.0692,0.1077,0.1692,0.236,0.3384,0.5,0.6812,0.8274,0.8812,0.9354,0.9735,0.9876,0.9994,1])
sp.toSimulation()

O.timingEnabled=True

O.engines=[
   ForceResetter(),
   InsertionSortCollider([Bo1_Sphere_Aabb(),Bo1_Facet_Aabb()]),
   InteractionLoop(
      [Ig2_Sphere_Sphere_ScGeom(),Ig2_Facet_Sphere_ScGeom()],
      [Ip2_FrictMat_FrictMat_FrictPhys()],
      [Law2_ScGeom_FrictPhys_CundallStrack()]
   ),
   NewtonIntegrator(gravity=(0,0,-9.81e2),damping=0.4),
   VTKRecorder(fileName='3d-vtk-',recorders=['all'],virtPeriod=0.25),
   PyRunner(command='checkUnbalanced_1()',realPeriod=2)
]
O.dt=0.5*PWaveTimeStep()

def checkUnbalanced_1():
   if O.time>2:
      O.engines=O.engines[:5]+[PyRunner(command='startPlating()',realPeriod=2)]

def startPlating():
   global time1
   time1=O.time
   print('startPlating')
   O.engines=O.engines[:5]+[TranslationEngine(translationAxis=(1,0,0),velocity=100,ids=rollerIds)]+[PyRunner(command='stopPlating()',realPeriod=2)]

def stopPlating():
   if O.time>(time1+13):
      global time2
      time2=O.time
      print('stopPlating')
      O.engines=O.engines[:5]+[TranslationEngine(ids=rollerIds)]+[PyRunner(command='checkUnbalanced_2()',realPeriod=2)]

sumVolume=0.0

def checkUnbalanced_2():
   if unbalancedForce()<.01 or O.time>(time2+1):
      O.pause()
      global sumVolume
      for b in O.bodies:
         if isinstance(b.shape, Sphere) and b.state.pos[2]<=90 and b.state.pos[2]>=0 and b.state.pos[0]<=1000 and b.state.pos[0]>=0 and b.state.pos[1]<=200 and b.state.pos[1]>=0:
            sumVolume+=4*pi*(b.shape.radius**3)/3
            plot.addData(i=b.id,radius=b.shape.radius,p_x=b.state.pos[0],p_y=b.state.pos[1],p_z=b.state.pos[2])
      print(sumVolume)
      plot.saveDataTxt('result.txt')
