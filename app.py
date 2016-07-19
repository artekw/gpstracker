from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_restful import reqparse, abort, Resource, Api
from datetime import datetime

app = Flask(__name__)
api = Api(app)

app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///points.db'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = ['False']
db = SQLAlchemy(app)


parser = reqparse.RequestParser()
parser.add_argument('latitude')
parser.add_argument('longitude')

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

    #def __repr__(self):
     #   return '<User %r>' % self.latitude


class Points(Resource):
    """REST"""
    def __init__(self):
        db.create_all()
        pass

    def get(self, point_id):
        pass

    def post(self):
        args = parser.parse_args()
        point = Coordinates(args['latitude'], args['longitude'], datetime.today())
        db.session.add(point)
        db.session.commit()
        return '', 201


##
## Actually setup the Api resource routing here
##
api.add_resource(Points, '/points')


if __name__ == '__main__':
    app.run(debug=True)
