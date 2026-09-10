import os
import json
import requests
from dotenv import load_dotenv

load_dotenv()

DESKTOP_IP = os.getenv("Cobra_API_IP_Port")
USERNAME = os.getenv("Cobra_API_USERNAME")
PASSWORD = os.getenv("Cobra_API_PASSWORD")

if not DESKTOP_IP:
    raise ValueError("IP Missing in .env")
if not USERNAME:
    raise ValueError("Username Missing in .env")
if not PASSWORD:
    raise ValueError("Password Missing in .env")

print(DESKTOP_IP, USERNAME, PASSWORD)

BASE_URL = f"http://{DESKTOP_IP}/api/v1.0"

def send_req(endpoint):
    url = f"{BASE_URL}{endpoint}"
    print(f"making request: {url}")

    try:
        response = requests.get(url, auth=(USERNAME, PASSWORD), timeout=10) # TODO: timeout matter?
        print(f"status code: {response.status_code}")

        response.raise_for_status()

        print(response.text)

        try:
            data = response.json()
            print(json.dumps(data, indent=2))

            return data
        except requests.exceptions.JSONDecodeError:
            print("response not JSON")
            return response.text
    except requests.exceptions.RequestException as e:
        print("request failed")
        print(e)
        return None
    
# send_req("/Jobs")
# send_req("/Jobs/&Maximum=20")
# print('\n')
send_req("/AuditLog")
# print('\n')