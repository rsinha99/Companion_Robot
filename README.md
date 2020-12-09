Alexa:

install ngrok
in ngrok folder run
./ngrok http 5000
leave this terminal open and run all of your other code in another terminal

the screen the pops up when you run the ngrok command will have an address that look like this: https://631df38f10d5.ngrok.io, copy that

Google https://developer.amazon.com/alexa/console/ask
make a new skill and name it whatever your robot's name is
go into your skill and click "invocation on the left" enter the invocation
then on the left click "interaction mode" and "intent"
add a new intent called exactly what the @ask.intent("") is called in the main.py script (this is case sensitive)
click "build model" at the top of the page
now on the left click "endpoint"
click "https"
paste the address you copied before into the top slot and select the second option on the drop down
click "save endpoints" at the top

To run:
python3 main.py
wait for text to show up 
say "Alexa tell 'robot name' to 'whatever you set the intent to be'
