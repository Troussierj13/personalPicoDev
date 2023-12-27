# A simple example that:
# - Connects to a WiFi Network defined by "ssid" and "password"
# - Performs a GET request (loads a webpage)
# - Queries the current time from a server

import network   # handles connecting to WiFi
import urequests # handles making and servicing network requests

# Connect to network
wlan = network.WLAN(network.STA_IF)
wlan.active(True)

# Fill in your network name (ssid) and password here:
ssid = 'Freebox-502A82'
password = 'ameria-viguistis&-alloquio-circumisse'
wlan.connect(ssid, password)


# Example 1. Make a GET request for google.com and print HTML
# Print the html content from google.com
print("1. Querying ennemy appear")
r = urequests.post("http://192.168.1.32:8000/api/game/ennemyAppear/6583163bda4906a86f3ebaea", data = "{'name': 'fzzfezfzef'}")
print(r.content)
r.close()
