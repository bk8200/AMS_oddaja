#!/usr/bin/python
# -*- coding: utf-8 -*-
from ams import *
from wmr import Wmr
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int64
from math import cos, sin, atan2
from world import MTAG
from graph_gen import tagPoses
import numpy as np

def wrapToPi(kot):
  kot = atan2(sin(kot),cos(kot))
  return kot  


class WmrRos(Wmr):
  def __init__(self):
    Wmr.__init__(self)
    
    self._pubOdom = rospy.Publisher('odom', Odometry, queue_size=10)
    self._subCmdVel = rospy.Subscriber('cmd_vel', Twist, self._handleCmdVel)
    self.subTag = rospy.Subscriber('tag',Int64,self._handleTag)
    self._subActions = rospy.Subscriber('path_actions', String, self._handleActions)
    self.encoderLOld = 0.0
    self.encoderROld = 0.0
    self.flag = 0
    self.x = 0.0
    self.y = 0.0
    self.fi = 0.0
    self.old_error = 0.0
    self.gama = 0.0
    self.D = 0.1207
    self.tagId = 0
    self.currentIdx = 0
    self.prepotovana_pot = 0.0    
    self.sled_rob = 'levo'
    self.new_path  = 1
    self.actions = []
    self.oldTag = 0

    
  def _handleCmdVel(self, msg):
    self.setVel(msg.linear.x, msg.angular.z)
    
  def _handleTag(self, msg):
    print(msg)
    self.tagId = MTAG.get(msg.data, self.tagId)

    
  def _handleEncoders(self, left, right, heading):
    if self.flag == 0:
      self.flag = 1
      self.encoderLOld = left 
      self.encoderROld = right


  
 
 
    # Resolucija enkoderja: 2^13-1
    max_gama = 8191.0
    # Vrednost enkoderja pri kotu 0 rad
    gama_0 = 5540.0
    # Pretvorba iz enkoderja v kot [rad]
    self.gama = - (heading - gama_0) * 2.0 * pi / max_gama
    self.gama = wrapToPi(self.gama)
    
    # Razlika levega in desnega enkoderja ob trenutkih k in k-1
    dl = -(left - self.encoderLOld)
    dr = right - self.encoderROld
    
    # Opravljena razdaljena v enkoderskem merilu
    d = (dl + dr) / 2.0
    
    # Pretvorba v opravljeno razdaljo v metrih 
    konst = 110445.0 # +- 200
    d = d / konst
    
    # Izračun spremebe po stanjih
    dx = d*cos(self.gama)*cos(self.fi)
    dy = d*cos(self.gama)*sin(self.fi)
    dfi = d*sin(self.gama) / self.D
    
    # Euler integracija stanj
    self.x = self.x + dx
    self.y = self.y + dy
    self.fi = self.fi + dfi

    if self.tagId != self.oldTag: 
      zk =  np.array( [tagPoses[self.tagId]] )#
      self.x  =zk[0][0] / 1000
      self.y  =zk[0][1] / 1000
      self.fi =zk[0][2]
      self.oldTag = self.tagId

    #print(self.x, self.y)
    # Izračun prepotovane poti od značke
    self.prepotovana_pot = self.prepotovana_pot + sqrt(dx**2 + dy**2)
    
    # Shranimo vrednosti enkoderjev za izračun razlik v naslednjem korku
    self.encoderLOld = left
    self.encoderROld = right 
    
    # Izpis meritev senzorjev in odometrije
    #print(self.x, heading, 360*self.gama/(2*pi), d)       
    #print( heading)
    #print(str(180*self.gama/pi) + '   ' +str(heading))
    #print(self.x, self.y, 360*self.fi/(2*pi), 360*self.gama/(2*pi), heading)
    #print(self.x, self.y, 180*self.fi/pi,180*self.gama/pi)
    #print(left, right, heading, 360*self.gama/(2*pi))
    
    
    msg = Odometry()
    #msg.header.stamp = t
    msg.header.frame_id = 'jaka/odom'
    msg.child_frame_id = 'jaka/wmr'
    msg.pose.pose.position.x = self.x
    msg.pose.pose.position.y = self.y
    msg.pose.pose.orientation = phiToQuaternionMsg(self.fi)
    
    msg.twist.twist.linear.x = d # delta_d
    msg.twist.twist.angular.z = self.gama # gama kot / heading
    
    self._pubOdom.publish(msg)
  
  def _handleLineSensor(self, left, right, sensors):
    # None, če ga ne zazna črte
    #  self.setVel(v, w)
    #  self.setWheelVel(left, right)
    '''
    K = 0.1
    if left == None or right == None:
      print('None')
    else:
      line_error = (left + right) / 2

        
      if abs(line_error) < 0.1:
        base_speed = 0.05
      else:
        base_speed = 0.0
       
      vv = K*abs(line_error) + base_speed
      if line_error > 0:
        self.setWheelVel(-vv/2, -vv)
        print('Desno kolo:' + str(vv))
      else:
        self.setWheelVel(-vv,-vv/2)
        print('Levo kolo:' + str(vv))
        '''
        
    # Sledenje v točko: smo na zadnjih 20 % prepotovane poti
    zadnjih_20_procentov = 0
    # Čakamo na actions 
    if self.actions != []:
      
      # Če je prvi node samo vzamemo vrednosti
      if self.currentIdx == 0:
        self.sled_rob = self.actions[self.currentIdx][0]
      # Preverjamo če smo dosegli next_Tag
      next_Tag = self.actions[self.currentIdx][2]     
      # Če je naslednji tag IMAGINAREN
      if next_Tag > 100: 
        #Popravilo tocke 105
        if  (self.actions[self.currentIdx][2] == 105) and (self.prepotovana_pot > self.actions[self.currentIdx][1]/1000.0 + 0.09):
          print('Imag reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0 
        #Popravilo tocke 106
        if  (self.actions[self.currentIdx][2] == 106) and (self.prepotovana_pot > self.actions[self.currentIdx][1]/1000.0 - 0.05):
          print('Imag reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0 
        #Popravilo tocke 109
        if  (self.actions[self.currentIdx][2] == 109) and (self.prepotovana_pot > self.actions[self.currentIdx][1]/1000.0 + 0.05):
          print('Imag reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0 
        #Popravilo tocke 108
        if  (self.actions[self.currentIdx][2] == 108) and (self.prepotovana_pot > self.actions[self.currentIdx][1]/1000.0 - 0.04):
          print('Imag reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0 
        elif self.prepotovana_pot > self.actions[self.currentIdx][1]/1000.0:
          print('Imag reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0 
        
        # Potovanje brez črte
        elif (next_Tag == 143):
          # Po 80% prepotovane poti začni iskati črto

          if self.prepotovana_pot > 0.8 * self.actions[self.currentIdx][1]/1000.0:
            if not (left == None or right == None) and right > 0 :
              print('Pot spet najdena')

              self.currentIdx = self.currentIdx + 1
              self.prepotovana_pot = 0.0 
        # Potovanje brez črte
        elif (next_Tag == 142):
          # Po 80% prepotovane poti začni iskati črto

          if self.prepotovana_pot > 0.8 * self.actions[self.currentIdx][1]/1000.0:
            print(left)
            if not (left == None or right == None) and left < 0 :
              print('Pot spet najdena')

              self.currentIdx = self.currentIdx + 1
              self.prepotovana_pot = 0.0 




      # Če je naslednji tag REALNE 
      elif next_Tag < 100 and next_Tag > 0 : # 
        
        if self.tagId == next_Tag:
          print('Real reached')
          self.currentIdx = self.currentIdx + 1
          self.prepotovana_pot = 0.0     

      if self.currentIdx >= len(self.actions):
        self.actions = []
        self.currentIdx = 0
        print('Konec poti')
      else:
        self.sled_rob = self.actions[self.currentIdx][0]
    else:
      self.sled_rob = 0
    


  
    if self.sled_rob == 'levo':
      line_error = left
      #next_Tag = self.actions[self.currentIdx][2]
      #if (next_Tag == 105):
      #  line_error = 1
    elif self.sled_rob == 'desno':
      line_error = right
      #next_Tag = self.actions[self.currentIdx][2]
      #if (next_Tag == 105):
      # line_error = 1


    sledenje_tock = False  
    if self.sled_rob == 'sredina':
      next_Tag = self.actions[self.currentIdx][2]
      if not ((next_Tag == 142) or (next_Tag == 143)):
        line_error = 0.5*(left + right)/2
        #print(line_error)
      else:
        
        
        #print( self.currentIdx )
        #print([tagPoses[ self.currentIdx[2] ] ])
        #print(self.actions)
        #print(self.actions[self.currentIdx])
        #print( self.actions[self.currentIdx][2])
        #print( [tagPoses[ self.actions[self.currentIdx][2] ] ])
        
        # Določimo koordinate točke v globalnem koordinatnem sistemu
        ref =  np.array( [tagPoses[ self.actions[self.currentIdx][2] ] ])
        x_ref = ref[0][0] / 1000
        y_ref = ref[0][1] / 1000
        
        #Popravilo tock 142 in 143 
        popravek = 0.05
        if  self.actions[self.currentIdx][2] == 142:
          x_ref = x_ref +popravek
          y_ref = y_ref +popravek
        if self.actions[self.currentIdx][2] == 143:
          x_ref = x_ref -popravek
          y_ref = y_ref +popravek
          
          
        # Parametri regulatorja
        v_max = 0.3
        Kw = 2.5
        Kgama = 4.0
        Kv = 2.0
        dist_min = 0.3
        
        # Razdalja do točke 
        dist_to = sqrt((self.x -x_ref)**2+(self.y -y_ref)**2)
      
        fi_ref = atan2((y_ref-self.y),(x_ref-self.x))
        
        e_fi = wrapToPi(fi_ref - self.fi)
        
        
        w = Kw * (e_fi)
        v = Kv * dist_to 
        
        # Omejimo na maksimalno hitrost
        if abs(v) > v_max:
          v = sign(v) * v_max
        
        # Pretvorba iz zadnje hitrosti na sprednjo
        vs = sqrt(w**2 * self.D**2 + v**2)
        
        gama_ref = atan2(w*self.D, v)
        
        e_gama = wrapToPi(gama_ref - self.gama)
        ws = Kgama * e_gama
      

        
        # Normalizacija na maksimalno hitrost
        vs = v_max * vs / (abs(vs)+abs(ws))
        
        # Implementacija žlajfanja
        brake = (dist_to/dist_min)**1/2
        if brake > 1:
          brake = 1
          
        ws = brake*ws
        vs = brake*vs*abs(cos(e_gama))

        sledenje_tock = True
        self.setVel(vs, ws)
           

    
    if left == None or right == None:
      # Ni vrednosti senzorja
      #print('None')
      pass
      
    elif  self.sled_rob != 0 and not sledenje_tock:
      
      # Parametri regulatroja sledenja črti
      Kwp = 2
      
      Kvd = 0.5
      Kvp = 0.1
      
      
      ws = Kwp * line_error
      vs = Kvp 
      #print(ws)
      self.setVel(vs, ws)
      #self.old_error = line_error
     
    
  def _handleActions(self, msg):
    print(msg.data)
    a = msg.data.split(';')
    if len(a):
      b = [x.split(',') for x in a]
      self.actions = [(x[0], float(x[1]), int(x[2])) for x in b if len(x) > 2] 
    if self.currentIdx == 0:
      self.sled_rob = self.actions[self.currentIdx][0]

  
  
if __name__ == '__main__':
  try:
    rospy.init_node('wmr')

    with WmrRos() as wmr:
      wmr.startUp()
      rate = rospy.Rate(50)
      while not rospy.is_shutdown():
        wmr.updateSensors()
        rate.sleep()
  except KeyboardInterrupt:
    pass
