#!/usr/bin/python2
# -*- coding: utf-8 -*-

from flask import Flask, jsonify, render_template, request, url_for, json
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

app = Flask(__name__)
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///points.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = ['False']
db = SQLAlchemy(app)

db.create_all()


class Coordinates(db.Model):
    """Model of database"""
    id = db.Column(db.Integer, primary_key=True)
    latitude = db.Column(db.Float)
    longitude = db.Column(db.Float)
    date = db.Column(db.DateTime) # datetime.date() object

    def __init__(self, latitude, longitude, date):
        self.latitude = latitude
        self.longitude = longitude
        self.date = date

    def __str__(self):
        return "{'latitude':%s, 'longitude':%s}" % (self.latitude, self.longitude)


def map_data():
    points = Coordinates.query.all()
    return [(p.date, p.latitude, p.longitude) for p in points]


@app.route('/', methods=["GET"])
@app.route('/query/', methods=["GET"])
def index():
    return render_template('index.html')


@app.route('/map', methods=["GET"])
def map():
    return render_template('map.html', data=map_data())


@app.route('/query/get', methods=["GET"])
def query():
    points = Coordinates.query.order_by(Coordinates.date)
    return render_template('points.html', points=points)


@app.route('/query/post', methods = ['POST'])
def post():
    if not request.json:
        abort(400)
    #print request.json
    point = Coordinates(request.json.get('latitude'), request.json.get('longitude'), datetime.today())
    db.session.add(point)
    db.session.commit()
    return '', 201


if __name__ == '__main__':
    app.run(debug=True)
