import pyrebase
import time
config = {
  #config removed for privacy
}

firebase = pyrebase.initialize_app(config)

db = firebase.database()
# db.child("users").child("Morty")
#data = {"name": "Mortimer 'Morty' Smith"}
#db.push(data)
while True:
	db.child("users").child("Morty").update({"name": "Mortimer 'Morty' Smith"+str(time.time())})
	time.sleep(0.1)