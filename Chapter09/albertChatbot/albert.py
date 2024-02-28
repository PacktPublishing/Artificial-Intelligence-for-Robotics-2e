import logging
import random
import re
from collections import namedtuple

log = logging.getLogger(__name__)

#
# Robot Personality Module
# part of Artificial Intelligence For Robotics
# 
# by Francis X Govers
# 
import numpy as np
from math import *
from sklearn.neighbors import NearestNeighbors
import rospy

class RobotEmotionEngine():
    def __init__(self):
        self.emostate = [90,0]
        self.emoText = "neutral 50"
        self.emotions = {
        "happy" : 50, "sad": 50,
        "welcome" : 50, "distant":50,
        "friend" : 50,"strange" :50,
        "curious" : 50,"frustrated":50,
        "fresh" : 50, "tired",50}
        self.bio = {"name":"Albert Tinman", "lastname": "Tinman", "age": "6 months",
        "maker": "granddad", "color": "green","food","electricity","author":"Isaac Asimov, of course",
        "school": "I do not go to school but I love to learn","hobby":"picking up toys", "job":"picking up toys"}
        # list of happy emotions and sad emotions
        self.emotBalance={"happy": "sad", "welcome":"distant",
           "friend": "strange", "curious": "frustrated","fresh": "tired"}
        self.emotionAxis{"happy":112, "welcome": 22,"friend":67,"curious":157,
                          "sad":292,"distant":202,"strange":247,"frustrated",337}
        self.update()
        
    def change(self,emot, val):
        self.emotions[emot]=val
        balance = 100 - val
        otherEmotion = self.emotBalance[emot]
        self.emotions[otherEmotion]=balance
        
    def update(self):
        rmin = 100
        rmax = 0
        thetamin =360
        thetamax=0
        for emote in self.emotions:
            theta = self.emotionAxis[emote]
            thetamax = min(theta,thetamax)
            thetamin = max(theta,thetamin)
            r = self.emotions[emote]
            rmin = max(rmin, r)
            rmax = max(rmax,r)
        stateR = (rmax-rmin)/ 2
        stateTheta = (thetamax-thetamin) / 2
        for emo in self.emotionAxis:
            thisAngle = self.emotionAxis[emo]
            if stateTheta > thisAngle
            myEmotion = emo
            break
        
        self.emostate = [stateTheta, stateR]
        if stateR < 55 and stateR > 45: 
            myEmotion = "neutral"
        self.emoText = myEmotion + " "+ str(stateR)
        print "Current Emotional State"  = myEmotion, stateR, stateTheta
        return
        
            
class HumanEmotionEngine():
    def __init__(self):
        self.emostate = [90,0]
        self.emoText = "neutral 50"
        self.emotions = {
        "happy" : 50, "sad": 50,
        "welcome" : 50, "distant":50}
        # list of happy emotions and sad emotions
        self.emotBalance={"happy": "sad", "welcome":"distant"}
        self.emotionAxis = {'distant': 315, 'welcome': 135, 'sad': 225, 'happy': 45}
        self.update()
        
    def change(self,emot, val):
        self.emotions[emot]=val
        balance = 100 - val
        otherEmotion = self.emotBalance[emot]
        self.emotions[otherEmotion]=balance
        
    def update(self):
        rmin = 100
        rmax = 0
        thetamin =360
        thetamax=0
        for emote in self.emotions:
            theta = self.emotionAxis[emote]
            thetamax = min(theta,thetamax)
            thetamin = max(theta,thetamin)
            r = self.emotions[emote]
            rmin = max(rmin, r)
            rmax = max(rmax,r)
        stateR = (rmax-rmin)/ 2
        stateTheta = (thetamax-thetamin) / 2
        for emo in self.emotionAxis:
            thisAngle = self.emotionAxis[emo]
            if stateTheta > thisAngle
            myEmotion = emo
            break
        
        self.emostate = [stateTheta, stateR]
        if stateR < 55 and stateR > 45: 
            myEmotion = "neutral"
        self.emoText = myEmotion + " "+ str(stateR)
        print "Current Emotional State"  = myEmotion, stateR, stateTheta
        return        
 

class HumanInformation():
    def __init__(self):
        self.info = {"name":"none"}
        self.info["age"]=0
        self.info["school"]="none"
        self.info["feeling"]="none"
        self.info["food"]="none"
        self.info["book"]="none"
        self.info["subject"]="none"
        self.info["song"]="none"
        self.info["teeth"]="none"
        self.info["jokes"]="none"
        # stuff is random information that we use to get more information and have the human answer questions
        # these are aimed at 3-7 year olds
        self.info["stuff"]="none"
        self.stuff = ["the color pink", "singing", "dancing", "dinosaurs", "race cars", "building things",
                      "robots", "airplaines", "space ships", "unicorns", "princesses"]
        self.points = self.info
        # setup points scoring scheme
        points = 20
        for item in self.points:
            self.points[item]=points
            points -= 2
            
    def setInfo(self,key,val):
        try:
            self.info[key] = val
            return 1
        except:
            print "Invalid Keywords in SetInfo ", key, val
            return 0
            
     
        
class ContextMemory():
    def __init__(self):
        self.currentContext = "None"
        self.currentHuman = None # pointer to the data file for the human we are currentl talking to
        self.humanFile = []
        self.emotion = "happy"
        self.humanEmotion = "happy"
        self.contextDict={}
        self.contextDict['currentHuman'] = self.currentHuman
        self.contextDict['robotEmotion'] = self.emotion
        self.contextDict['humanEmotion'] = self.humanEmotion
        
    def inContext(self, datum):
        if datum in self.contextDict:
            return self.contextDict[datum]
        else:
            return 0
            
    def setHuman(self,human):
        self.currentHuman = human
        self.humanFile.append(human)  # add this person to the database of people we know
        
    def addHuman(self,human):
        self.humanFile.append(human)  # add this person to the database
        # used at startup to recall humans we have met before
        
    
       
        
        
        
        
# interface to send and receive commands to the robot        
class RosIF():
    def __init__(self):
        self.lastUpdate = 0.0
        rospy.Subscriber("robotCommand",String,self.robCom_callback)
        self.robotCommand = rospy.Publisher("robotCommand",String,queue_size=10)
    

    def command(self,cmd):
        rospy.loginfo(cmd)
        self.robotCommand.publish(cmd)
        self.frame.statusBox.addItem(cmd)
		
    def robCom_callback(self,cmd):
        rospy.loginfo(cmd)
        robot_command = cmd.data
        # received command for robot - process



class Key:
    def __init__(self, word, weight, decomps):
        self.word = word
        self.weight = weight
        self.decomps = decomps


class Decomp:
    def __init__(self, parts, save, reasmbs):
        self.parts = parts
        self.save = save
        self.reasmbs = reasmbs
        self.next_reasmb_index = 0


class Eliza:
    def __init__(self):
        self.initials = []
        self.finals = []
        self.quits = []
        self.pres = {}
        self.posts = {}
        self.synons = {}
        self.keys = {}
        self.memory = []

    def load(self, path):
        key = None
        decomp = None
        with open(path) as file:
            for line in file:
                if not line.strip():
                    continue
                tag, content = [part.strip() for part in line.split(':')]
                if tag == 'initial':
                    self.initials.append(content)
                elif tag == 'final':
                    self.finals.append(content)
                elif tag == 'quit':
                    self.quits.append(content)
                elif tag == 'pre':
                    parts = content.split(' ')
                    self.pres[parts[0]] = parts[1:]
                elif tag == 'post':
                    parts = content.split(' ')
                    self.posts[parts[0]] = parts[1:]
                elif tag == 'synon':
                    parts = content.split(' ')
                    self.synons[parts[0]] = parts
                elif tag == 'key':
                    parts = content.split(' ')
                    word = parts[0]
                    weight = int(parts[1]) if len(parts) > 1 else 1
                    key = Key(word, weight, [])
                    self.keys[word] = key
                elif tag == 'decomp':
                    parts = content.split(' ')
                    save = False
                    if parts[0] == '$':
                        save = True
                        parts = parts[1:]
                    decomp = Decomp(parts, save, [])
                    key.decomps.append(decomp)
                elif tag == 'reasmb':
                    parts = content.split(' ')
                    decomp.reasmbs.append(parts)

    def _match_decomp_r(self, parts, words, results):
        if not parts and not words:
            return True
        if not parts or (not words and parts != ['*']):
            return False
        if parts[0] == '*':
            for index in range(len(words), -1, -1):
                results.append(words[:index])
                if self._match_decomp_r(parts[1:], words[index:], results):
                    return True
                results.pop()
            return False
        elif parts[0].startswith('@'):
            root = parts[0][1:]
            if not root in self.synons:
                raise ValueError("Unknown synonym root {}".format(root))
            if not words[0].lower() in self.synons[root]:
                return False
            results.append([words[0]])
            return self._match_decomp_r(parts[1:], words[1:], results)
        elif parts[0].lower() != words[0].lower():
            return False
        else:
            return self._match_decomp_r(parts[1:], words[1:], results)

    def _match_decomp(self, parts, words):
        results = []
        if self._match_decomp_r(parts, words, results):
            return results
        return None

    def _next_reasmb(self, decomp):
        index = decomp.next_reasmb_index
        result = decomp.reasmbs[index % len(decomp.reasmbs)]
        decomp.next_reasmb_index = index + 1
        return result

    def _reassemble(self, reasmb, results):
        output = []
        for reword in reasmb:
            if reword[0] == '(' and reword[-1] == ')':
                index = int(reword[1:-1])
                if index < 1 or index > len(results):
                    raise ValueError("Invalid result index {}".format(index))
                insert = results[index - 1]
                for punct in [',', '.', ';']:
                    if punct in insert:
                        insert = insert[:insert.index(punct)]
                output.extend(insert)
            else:
                output.append(reword)
        return output

    def _sub(self, words, sub):
        output = []
        for word in words:
            word_lower = word.lower()
            if word_lower in sub:
                output.extend(sub[word_lower])
            else:
                output.append(word)
        return output

    def _match_key(self, words, key):
        for decomp in key.decomps:
            results = self._match_decomp(decomp.parts, words)
            if results is None:
                log.debug('Decomp did not match: %s', decomp.parts)
                continue
            log.debug('Decomp matched: %s', decomp.parts)
            log.debug('Decomp results: %s', results)
            results = [self._sub(words, self.posts) for words in results]
            log.debug('Decomp results after posts: %s', results)
            reasmb = self._next_reasmb(decomp)
            log.debug('Using reassembly: %s', reasmb)
            if reasmb[0] == 'goto':
                goto_key = reasmb[1]
                if not goto_key in self.keys:
                    raise ValueError("Invalid goto key {}".format(goto_key))
                log.debug('Goto key: %s', goto_key)
                return self._match_key(words, self.keys[goto_key])
            output = self._reassemble(reasmb, results)
            if decomp.save:
                self.memory.append(output)
                log.debug('Saved to memory: %s', output)
                continue
            return output
        return None

    def respond(self, text):
        if text in self.quits:
            return None

        text = re.sub(r'\s*\.+\s*', ' . ', text)
        text = re.sub(r'\s*,+\s*', ' , ', text)
        text = re.sub(r'\s*;+\s*', ' ; ', text)
        log.debug('After punctuation cleanup: %s', text)

        words = [w for w in text.split(' ') if w]
        log.debug('Input: %s', words)

        words = self._sub(words, self.pres)
        log.debug('After pre-substitution: %s', words)

        keys = [self.keys[w.lower()] for w in words if w.lower() in self.keys]
        keys = sorted(keys, key=lambda k: -k.weight)
        log.debug('Sorted keys: %s', [(k.word, k.weight) for k in keys])

        output = None

        for key in keys:
            output = self._match_key(words, key)
            if output:
                log.debug('Output from key: %s', output)
                break
        if not output:
            if self.memory:
                index = random.randrange(len(self.memory))
                output = self.memory.pop(index)
                log.debug('Output from memory: %s', output)
            else:
                output = self._next_reasmb(self.keys['xnone'].decomps[0])
                log.debug('Output from xnone: %s', output)

        return " ".join(output)

    def initial(self):
        return random.choice(self.initials)

    def final(self):
        return random.choice(self.finals)

    def run(self):
        print(self.initial())

        while True:
            sent = input('> ')

            output = self.respond(sent)
            if output is None:
                break

            print(output)

        print(self.final())


def main():
    eliza = Eliza()
    eliza.load('AlbertPersonality.txt')
    eliza.run()

if __name__ == '__main__':
    logging.basicConfig()
    main()