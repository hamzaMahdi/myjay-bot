import pyrebase
import time
config = {
  #config removed for privacy
}

firebase = pyrebase.initialize_app(config)

db = firebase.database()

# while True:
# 	user = db.child("users").get()
# 	print(user.val()) # users

def stream_handler(message):
    print(message["event"]) # put
    print(message["path"]) # /-K7yGTTEp7O549EzTYtI
    print(message["data"].get('name')) # {'title': 'Pyrebase', "body": "etc..."}
    


my_stream = db.child("users").stream(stream_handler)

