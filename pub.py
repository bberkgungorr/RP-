import paho.mqtt.client as paho
import time
import sys
from tkinter import*



pencere = Tk()        


def sekiz():
    otomatik_kontrol_pencere = Toplevel(pencere)
    baslik = otomatik_kontrol_pencere.title("Otomatik Kontrol")
    otomatik_kontrol_pencere.geometry("400x300")
    def start():
        client.publish("berk/otokontrol/başlat/bitir", payload="başlat")
        

    def stop():
        client.publish("berk/otokontrol/başlat/bitir", payload="bitir")
        otomatik_kontrol_pencere.destroy()
        
    
    startButton = Button(otomatik_kontrol_pencere, text="Otomatik Kontrolü Başlat", command=start)
    stopButton = Button(otomatik_kontrol_pencere, text="Otomatik Kontrolü Durdur", command=stop)       
    startButton.place(relx=0.0,rely=0.0,relheight=0.5,relwidth=1)
    stopButton.place(relx=0.0,rely=0.5,relheight=0.5,relwidth=1)
    


def onbir():
    manuel_kontrol_pencere = Toplevel(pencere)
    baslik = manuel_kontrol_pencere.title("Manuel Kontrol")
    manuel_kontrol_pencere.geometry("700x500")
    
    def bir():
        client.publish("berk/servo/2.5", payload="servo2.5")
        

    def iki():
        client.publish("berk/servo/7.5", payload="servo7.5")
 
    def üç():
        client.publish("berk/servo/12.5", payload="servo12.5")
           
    
    def beş():
        client.publish("berk/fan/60", payload="fan60")
    
    
    def altı():
        client.publish("berk/fan/75", payload="fan75")
    
    def yedi():
        client.publish("berk/fan/100", payload="fan100")


    def dokuz():
        Servo_ayar = int(Num1.get())
        ayarlanacak_deger = Servo_ayar//18 + 2.5
        client.publish("berk/servo/kullanıcı", payload=ayarlanacak_deger)
        time.sleep(1)
        Entry_servo.delete(0,END)

    def on():
        fan_ayar = int(Num2.get())
        client.publish("berk/fan/kullanıcı", payload=fan_ayar)
        Entry_fan.delete(0,END)


    def pıd():
        pıd_derece = int(Num3.get())
        client.publish("berk/pıd", payload=pıd_derece)
        
    def cıkıs():
        manuel_kontrol_pencere.destroy()
        

    Num1 = StringVar()
    Num2 = StringVar()
    Num3 = StringVar()

    button1 = Button(manuel_kontrol_pencere, text = " KAPAT " , command=cıkıs)
    button1.place(relx=0.0,rely=0.4,relheight=0.1,relwidth=1)

    button6 = Button(manuel_kontrol_pencere, text = " fan hızını %60 yap " , width=20, command=beş)
    button6.place(relx=0.2,rely=0.0,relheight=0.1,relwidth=0.2)

    button7 = Button(manuel_kontrol_pencere, text = " fan hızını %75 yap " , width=20, command=altı)
    button7.place(relx=0.2,rely=0.1,relheight=0.1,relwidth=0.2)

    button8 = Button(manuel_kontrol_pencere, text = " fan hızını %100 yap " , width=20, command=yedi)
    button8.place(relx=0.2,rely=0.2,relheight=0.1,relwidth=0.2)

    button2 = Button(manuel_kontrol_pencere, text = " servoyu tam sol yap " , command=bir)
    button2.place(relx=0.0,rely=0.0,relheight=0.1,relwidth=0.2)
 
    button3 = Button(manuel_kontrol_pencere, text = " servoyu tam ortala " , width=20, command=iki)
    button3.place(relx=0.0,rely=0.1,relheight=0.1,relwidth=0.2)
 
    button4 = Button(manuel_kontrol_pencere, text = " servoyu tam sağ yap " , width=20, command=üç)
    button4.place(relx=0.0,rely=0.2,relheight=0.1,relwidth=0.2)

    Entry_servo = Entry(manuel_kontrol_pencere, textvariable = Num1)
    Entry_servo.place(relx=0.5,rely=0.1,relheight=0.1,relwidth=0.2)
    button10 = Button(manuel_kontrol_pencere, text = " servo motoru ayarla" , width=20, command=dokuz)
    button10.place(relx=0.7,rely=0.1,relheight=0.1,relwidth=0.2)

    Entry_fan = Entry(manuel_kontrol_pencere, textvariable = Num2)
    Entry_fan.place(relx=0.5,rely=0.0,relheight=0.1,relwidth=0.2)
    button11 = Button(manuel_kontrol_pencere, text = " fan hızını ayarla" , width=20, command=on)
    button11.place(relx=0.7,rely=0.0,relheight=0.1,relwidth=0.2)

    button14 = Button(manuel_kontrol_pencere, text = " dereceyi ayarlayın " , width=20, command=pıd)
    button14.place(relx=0.7,rely=0.2,relheight=0.1,relwidth=0.2)
    Entry_pıd = Entry(manuel_kontrol_pencere, textvariable = Num3)
    Entry_pıd.place(relx=0.5,rely=0.2,relheight=0.1,relwidth=0.2)

    
    
def sıcaklıkdegeri(client,userdata,msg):
    print(float(msg.payload))
    E1.insert(0,float(msg.payload)) 
            

def toprak_nem():
    client.publish("berk/toprak_nem", "toprak_nem")



baslik = pencere.title("Sera Otomasyonu")
pencere.geometry("700x400")

button13 = Button(pencere, text = " topraktaki su durumu " , width=20, command=toprak_nem)
button13.place(relx=0.0,rely=0.5,relheight=0.1,relwidth=0.2)


button9 = Button(pencere, text = " otomatik kontrol " , width=20, command=sekiz)
button9.place(relx=0.0,rely=0.0,relheight=0.2,relwidth=1)

button11 = Button(pencere, text = " manuel kontrol " , width=20, command=onbir)
button11.place(relx=0.0,rely=0.2,relheight=0.2,relwidth=1)

button12 = Button(pencere, text = " KAPAT " , command=exit)
button12.place(relx=0.0,rely=0.6,relheight=0.1,relwidth=1)

etiket = Label(text="°C")
etiket.place(relx=0.4,rely=0.4,relheight=0.1,relwidth=0.05)

E1 = Entry(pencere)
E1.place(relx=0.2,rely=0.4,relheight=0.1,relwidth=0.2)

client = paho.Client()
client.connect("iot.eclipse.org", port=1883, keepalive=60)
client.message_callback_add("berk/sıcaklık/degeri",sıcaklıkdegeri)
client.subscribe("berk/sıcaklık/degeri", qos=1)

pencere.mainloop()

client.loop_forever()
