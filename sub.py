from tkinter import*
from tkinter import messagebox
import RPi.GPIO as GPIO
import time
import sys
import threading
import paho.mqtt.client as paho
import Adafruit_DHT

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

sensor = Adafruit_DHT.DHT11
pin = 17

input1 = 16
input2 = 18
enable = 22
water_pump1 = 13
water_pump2 = 15
servo = 3
water_sensor = 37
channel = 40



GPIO.setup(water_sensor, GPIO.IN)
GPIO.setup(channel, GPIO.IN)

GPIO.setup(servo,GPIO.OUT)
GPIO.setup(input1,GPIO.OUT)
GPIO.setup(input2,GPIO.OUT)
GPIO.setup(enable,GPIO.OUT)
GPIO.setup(water_pump1,GPIO.OUT)
GPIO.setup(water_pump2,GPIO.OUT)

pwm_servo = GPIO.PWM(servo,50)              
pwm_servo.start(0)
pwm_fan = GPIO.PWM(enable,50)              
pwm_fan.start(0)

running = None
pıd_degeri = 0
otomatik = ""


def on_connect(client, userdata, flags, rc):
  print("Connected with result code "+str(rc))

def fan(client, userdata, msg):
    GPIO.output(input1, True)
    GPIO.output(input2, False)
    pwm_fan.ChangeDutyCycle(int(msg.payload))                   
    GPIO.output(enable, True)


def servo(client, userdata, msg):
  pwm_servo.ChangeDutyCycle(float(msg.payload))


def toprak(client, userdata, msg):
    #GPIO.add_event_detect(channel, GPIO.BOTH, bouncetime=100) 
    #GPIO.add_event_callback(channel, callback) 
    if (GPIO.input(channel) == 1):
        print("Toprakta su yok!")
        time.sleep(0.1)
        GPIO.output(water_pump1, True)
        GPIO.output(water_pump2, False)
        time.sleep(0.1)
        GPIO.output(water_pump1, False)
        GPIO.output(water_pump2, False)
        
        

    else:
        print("Toprakta su var!")
        time.sleep(1)
        GPIO.output(water_pump1, False)
        GPIO.output(water_pump2, False)
        time.sleep(1)
                



def pıd(client, userdata, msg):
    global pıd_degeri
    pıd_degeri = int(msg.payload)
    PID()
    

class PID():
    def __init__(self, P=9, I=3 , D=1):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.0001
        self.current_time = time.time()
        self.last_time = self.current_time
            
        self.clear()

    def clear(self):
        self.SetPoint = pıd_degeri
            
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
            
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

        self.update()

    def update(self):
        humidity11, temp11 = Adafruit_DHT.read_retry(11, 17)
        time.sleep(10)
        feedback_value = temp11            
            
        error =   feedback_value - self.SetPoint
           
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        print(self.current_time)
        print(self.last_time)
        print(delta_time)
        delta_error = error - self.last_error
        print(self.last_error)
        print(error)
        print("yukarıdaki fark")
        print(delta_error)

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            print(self.Kp)
            print(error)
            print(self.PTerm)
            print(delta_time)
            print(self.ITerm)

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
                print(self.DTerm)

            self.last_time = self.current_time
            self.last_error = error
            print(self.last_error)

            

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            if self.output <= 0 :
                self.output = 0
            elif self.output >= 100 :
                self.output = 100
            print(self.output)

            if (feedback_value != self.SetPoint):
                pwm_fan.ChangeDutyCycle(self.output)
                self.update()
                time.sleep(5)
                    
                    
            else:
                pwm_fan.ChangeDutyCycle(0)
                    
               
                
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

def fan_100(client, userdata, msg):
    GPIO.output(input1, True)
    GPIO.output(input2, False)
    pwm_fan.ChangeDutyCycle(100)                   
    GPIO.output(enable, True)
  

def fan_75(client, userdata, msg):
    GPIO.output(input1, True)
    GPIO.output(input2, False)
    pwm_fan.ChangeDutyCycle(75)                   
    GPIO.output(enable, True)

def fan_60(client, userdata, msg):
    GPIO.output(input1, True)
    GPIO.output(input2, False)
    pwm_fan.ChangeDutyCycle(60)                   
    GPIO.output(enable, True)

def servo_12(client, userdata, msg):
    pwm_servo.ChangeDutyCycle(12.5)

def servo_7(client, userdata, msg):
    pwm_servo.ChangeDutyCycle(7.5)

def servo_2(client, userdata, msg):
    pwm_servo.ChangeDutyCycle(2)

def start(client, userdata, msg):
    global otomatik
    otomatik = msg.payload.decode()
    if otomatik == "başlat":
        oto_kont()
    elif otomatik == "bitir":
        pwm_servo.ChangeDutyCycle(2)
        GPIO.output(input1, True)
        GPIO.output(input2, False)
        pwm_fan.ChangeDutyCycle(0)                   
        GPIO.output(enable, True)
        GPIO.output(water_pump1, False)
        GPIO.output(water_pump2, False)

def sıcaklık(client, userdata, msg):
    humidity11, temp11 = Adafruit_DHT.read_retry(11, 17)
    client.publish("berk/sıcaklık/degeri", payload=temp11)
    print(temp11)

def oto_kont():   
            
    humidity11, temp11 = Adafruit_DHT.read_retry(11, 17)
    print("Temperature: %d C" % temp11)
    if (GPIO.input(water_sensor)==1):
        print("No Rain Detected")
        if temp11 != 0  and temp11 >= 20 and temp11 < 22:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(60)                   
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(5)            
            

        elif temp11 != 0  and temp11 >= 22 and temp11 < 24:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(75)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(7.5)
                

        elif temp11 != 0  and temp11 >= 24 and temp11 < 26:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(90)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(10)
            

        elif temp11 != 0  and temp11 >= 26:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(100)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(12.5)
                    
        
        elif temp11 < 20 and temp11 != 0:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(45)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(2.5)
                
        elif (GPIO.input(channel)==1):
            print("Toprakta su yok!")
            GPIO.output(water_pump1, True)
            GPIO.output(water_pump2, False)
            time.sleep(2)
            GPIO.output(water_pump1, False)
            GPIO.output(water_pump2, False)
                

        elif(GPIO.input(channel)==0):
            print("Toprakta su var!")
            GPIO.output(water_pump1, False)
            GPIO.output(water_pump2, False)
            time.sleep(2)
                    
    elif(GPIO.input(water_sensor)==0):
        print("Rain Detected")
        if temp11 != 0  and temp11 >= 20 and temp11 < 22:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(60)                   
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(2.5)            
            

        elif temp11 != 0  and temp11 >= 22 and temp11 < 24:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(75)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(2.5)
                

        elif temp11 != 0  and temp11 >= 24 and temp11 < 26:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(90)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(2.5)
            

        elif temp11 != 0  and temp11 >= 26:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(100)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(2.5)
                    
        
        elif temp11 < 20 and temp11 != 0:
            GPIO.output(input1, True)
            GPIO.output(input2, False)
            pwm_fan.ChangeDutyCycle(45)                  
            GPIO.output(enable, True)
            pwm_servo.ChangeDutyCycle(2.5)
                
        elif (GPIO.input(channel)==1):
            print("Toprakta su yok!")
            GPIO.output(water_pump1, True)
            GPIO.output(water_pump2, False)
            time.sleep(2)
            GPIO.output(water_pump1, False)
            GPIO.output(water_pump2, False)
                

        elif(GPIO.input(channel)==0):
            print("Toprakta su var!")
            GPIO.output(water_pump1, False)
            GPIO.output(water_pump2, False)
            time.sleep(2)
        
    time.sleep(10)
    start(client, userdata==None,otomatik)


 
client = paho.Client()
client.connect("iot.eclipse.org",1883,60)

client.on_connect = on_connect
client.message_callback_add("berk/fan/kullanıcı", fan)
client.message_callback_add("berk/servo/kullanıcı", servo)
client.message_callback_add("berk/toprak_nem", toprak)
client.message_callback_add("berk/pıd", pıd)
client.message_callback_add("berk/fan/100", fan_100)
client.message_callback_add("berk/fan/75", fan_75)
client.message_callback_add("berk/fan/60", fan_60)
client.message_callback_add("berk/servo/12.5", servo_12)
client.message_callback_add("berk/servo/7.5", servo_7)
client.message_callback_add("berk/servo/2.5", servo_2)
client.message_callback_add("berk/otokontrol/başlat/bitir", start)


client.subscribe("berk/fan/kullanıcı", qos=1)
client.subscribe("berk/servo/kullanıcı", qos=1)
client.subscribe("berk/toprak_nem", qos=1)
client.subscribe("berk/pıd", qos=1)
client.subscribe("berk/fan/100", qos=1)
client.subscribe("berk/fan/75", qos=1)
client.subscribe("berk/fan/60", qos=1)
client.subscribe("berk/servo/12.5", qos=1)
client.subscribe("berk/servo/7.5", qos=1)
client.subscribe("berk/servo/2.5", qos=1)
client.subscribe("berk/otokontrol/başlat/bitir", qos=1)




client.loop_forever()

