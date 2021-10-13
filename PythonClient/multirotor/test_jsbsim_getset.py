import setup_path
import airsim

client = airsim.MultirotorClient()
client.confirmConnection()

alt = client.getJSBSimProperty("position/geod-alt-ft")

print(f"alt: {alt}")

client.setJSBSimProperty("position/geod-alt-ft", 1)

alt = client.getJSBSimProperty("position/geod-alt-ft")

print(f"alt: {alt}")


