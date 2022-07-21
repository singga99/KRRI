from pydoc import cli
from pytube import YouTube
import playsound
import moviepy.editor as mp
import schedule
import time

## youtube download
path = "/home/krri/shin_dev/happy_krri"
# url = "https://www.youtube.com/watch?v=KM7yPimuGGQ"

# yt =  YouTube(url)
# stream = yt.streams.get_highest_resolution()
# stream.download(path)

## audio 추출
# clip = mp.VideoFileClip(path + "/10minutes_test.mp4")
# clip.audio.write_audiofile(path + "/10minutes_test.wav")

def Lunch():
    print("-----Play Start-----")
    playsound.playsound(path + "/lunch_stretching.wav")
    print("-----Lunch Time-----")

def LeaveWork():
    print("-----Play Start-----")
    playsound.playsound(path + "/leave_work.wav")
    print("-----Leave Work-----")
    
schedule.every().day.at("11:20").do(Lunch)
schedule.every().day.at("17:50").do(LeaveWork)

while True:
    schedule.run_pending()
    time.sleep(1)