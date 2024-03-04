from yade import pack, ymport, timing, plot

O.materials.append(FrictMat(density=4500,label='TC4'))

workBenchIds=O.bodies.append(ymport.stl('workbench.stl',wire=True))
rollerIds=O.bodies.append(ymport.stl('roller.stl',wire=True))
baseIds=O.bodies.append(ymport.stl('base.stl',wire=True))

sp=pack.SpherePack()
sp.makeCloud((0,0,15),(10,30,-9),psdSizes=[0.264,0.3,0.353,0.4,0.462,0.533,0.6,0.7],psdCumm=[0,0.1023,0.2575,0.4116,0.59,0.7562,0.884,1])
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
   NewtonIntegrator(gravity=(0,0,-9.81),damping=0.4),
   VTKRecorder(fileName='3d-vtk-',recorders=['all'],virtPeriod=0.25),
   PyRunner(dead=True,iterPeriod=1,command='updateKinematicEngines()',label='updateEngines'),
   CombinedKinematicEngine(dead=True,ids=rollerIds,label='combEngine')+TranslationEngine(translationAxis=(1,0,0),velocity=5)+RotationEngine(rotationAxis=(0,-1,0),angularVelocity=0.1*pi,rotateAroundZero=True,zeroPoint=(-10,0,11.2)),
   PyRunner(command='checkUnbalanced_1()',realPeriod=2)
]
O.dt=0.5*PWaveTimeStep()

transEngine,rotEngine=combEngine.comb[0],combEngine.comb[1]

def updateKinematicEngines():
   v=Vector3(1,0,0)
   rotEngine.zeroPoint+=v*5*O.dt

def checkUnbalanced_1():
   if unbalancedForce()<.05:
      global time1
      time1=O.time
      O.engines=O.engines[:7]+[TranslationEngine(translationAxis=(0,0,1),velocity=2,ids=baseIds)]+[PyRunner(command='stopBase()',realPeriod=2)]

def stopBase():
   if O.time>(time1+5):
      O.engines=O.engines[:7]+[TranslationEngine(ids=baseIds)]+[PyRunner(command='startPlating()',realPeriod=2)]

def startPlating():
   global time2
   time2=O.time
   O.engines=O.engines[:7]+[PyRunner(command='stopPlating()',realPeriod=2)]
   updateEngines.dead=False
   combEngine.dead=False

def stopPlating():
   if O.time>(time2+18):
      global time3
      time3=O.time
      O.engines=O.engines[:5]+[TranslationEngine(ids=rollerIds)]+[TranslationEngine(translationAxis=(0,0,-1),velocity=2,ids=baseIds)]+[PyRunner(command='unloadBase()',realPeriod=2)]

def unloadBase():
   if O.time>(time3+5):
      global time4
      time4=O.time
      O.engines=O.engines[:5]+[TranslationEngine(ids=baseIds)]+[TranslationEngine(translationAxis=(1,0,0),velocity=5,ids=rollerIds)]+[PyRunner(command='unloadPlate()',realPeriod=2)]

def unloadPlate():
   if O.time>(time4+4):
      global time5
      time5=O.time
      O.engines=O.engines[:5]+[TranslationEngine(ids=rollerIds)]+[PyRunner(command='checkUnbalanced_2()',realPeriod=2)]

sumVolume=0.0

def checkUnbalanced_2():
   if unbalancedForce()<.05 or O.time>(time5+5):
      O.pause()
      global sumVolume
      for b in O.bodies:
         if isinstance(b.shape, Sphere) and b.state.pos[2]<=1.2 and b.state.pos[2]>=0 and b.state.pos[0]<=80 and b.state.pos[0]>=10 and b.state.pos[1]<=30 and b.state.pos[1]>=0:
            sumVolume+=4*pi*(b.shape.radius**3)/3
            plot.addData(i=b.id,radius=b.shape.radius,p_x=b.state.pos[0],p_y=b.state.pos[1],p_z=b.state.pos[2])
      print(sumVolume)
      plot.saveDataTxt('result.txt')
