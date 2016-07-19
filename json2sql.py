import sqlite3
import json


with open('json-data.json', 'rb') as json_file:
    j = json.load(json_file)

for i in range(len(j)):
    print "item%s = Coordinates('%s', '%s', '%s')" % (i,j[i]['latitude'],j[i]['longitude'], j[i]['timestamp'])
    print "db.session.add(item%s)" % (i)
