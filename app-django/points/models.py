from __future__ import unicode_literals

from django.db import models
from django.utils import timezone

# Create your models here.

class Coordinates(models.Model):
	latitude = models.DecimalField(max_digits=11, decimal_places=7)
	longitude = models.DecimalField(max_digits=11, decimal_places=7)


	def publish(self):
		self.published_date = timezone.now()
		self.save()


	def __str__(self):
		return "%f x %f" % (self.longitude, self.latitude)


"""
class User(models.Model):
	login
	password
"""