#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import String
import logging
import time
from threading import Thread, Event
from lib import Microphone
import requests
import datetime
import hmac
import urllib3
import urllib
import json
import hashlib
import base64
import sys
import string
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')


def to_md5_base64(strBody):
    hash = hashlib.md5()
    hash.update(strBody)
    m = hash.digest().encode('base64').strip()
    hash = hashlib.md5()
    hash.update(m)
    return hash.digest().encode('base64').strip()


def to_sha1_base64(stringToSign, secret):
    hmacsha1 = hmac.new(str(secret), str(stringToSign), hashlib.sha1)
    return base64.b64encode(hmacsha1.digest())


def stt(data):
    tmp = []
    for d in data:
        tmp.append(d)
    audio = ''.join(tmp)
    date = datetime.datetime.strftime(
        datetime.datetime.utcnow(), "%a, %d %b %Y %H:%M:%S GMT")

    options = {
        'url': 'https://nlsapi.aliyun.com/recognize?model=chat',
        'method': 'POST',
        'body': audio,
    }
    headers = {
        'Authorization': '',
        'Content-type': 'audio/pcm; samplerate=16000',
        'Accept': 'application/json',
        'Date': date,
        'Content-Length': str(len(audio))
    }
    ak_id='LTAILDQJlOiQl0qw'
    ak_secret='chX2iGle2jTFpikrNEFN0EQT9mIRU3'

    body = options['body']

    bodymd5 = to_md5_base64(body)

    stringToSign = options['method'] + '\n' + \
    headers['Accept'] + '\n' + bodymd5 + '\n' + \
    headers['Content-type'] + '\n' + headers['Date']
    signature = to_sha1_base64(stringToSign, ak_secret)

    authHeader = 'Dataplus ' + ak_id + ':' + signature
    headers['Authorization'] = authHeader
    url = options['url']
    r = requests.post(url, body, headers=headers, verify=False)
    text = ''
    try:        
        if 'result' in r.json():
            text = r.json()['result'].encode('utf-8')
    except requests.exceptions.HTTPError:
        rospy.loginfo('Request failed with response: %r',r.text)

    print r.json()
    return text


def main():
    logging.basicConfig(level=logging.INFO)
    quit_event = Event()
    mic = Microphone(quit_event=quit_event)
    pub = rospy.Publisher('voice/stt', String, queue_size=10)
    rospy.init_node('ali_asr', anonymous=True)
    while not rospy.is_shutdown():
        data = mic.listen()
        text = stt(data)
        hello_str = "REC: %s" % text
        rospy.loginfo(hello_str)
        if text:
            pub.publish(text)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
