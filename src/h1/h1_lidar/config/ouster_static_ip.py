import requests

r = requests.put('http://os-122537003417.local/api/v1/system/network/ipv4/override', headers={'Content-Type': 'application/json'}, json='192.168.123.20/24')
print(r)
print(r.content)