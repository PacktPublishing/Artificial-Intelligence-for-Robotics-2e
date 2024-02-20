from mycroft import MycroftSkill, intent_file_handler
from random import choice


class Knockknock(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)

    @intent_file_handler('knockknock.intent')
    def handle_knockknock(self, message):
        #self.speak_dialog('knockknock')
	name,punchline = self.pick_joke()
	response=self.get_response(announcement="knock, knock")
	# response will always be "who's there"
	response=self.get_response(announcement=name)
	# response will be "name who"
	# if end of respose is not the word who, we can re-prompt
	if "who" not in response:
		prompt = "You are supposed to say "+name+" who"
		response=self.get_response(announcement=prompt)
	self.speak(punchline)
	
    def pick_joke():
	jokeFile="knockknock.jokes"
	jfile = open(jokeFile,"r")
	jokes = []
	for jokeline in jfile:
		jokes.append(jokeline)
	joke = choice(jokes)
	jokeParts = joke.split("/")
	name = jokeParts[0]
	punchline = jokeParts[1]
	return name, punchline	

def create_skill():
    return Knockknock()

