from django.db import models


class Pose (models.Model):
    # To be used by Scene and GoalSet

    # Denormalize the postion and orientation for simplicity

    position_x = models.FloatField(default=0)
    position_y = models.FloatField(default=0)
    position_z = models.FloatField(default=0)

    orientation_x = models.FloatField(default=0)
    orientation_y = models.FloatField(default=0)
    orientation_z = models.FloatField(default=0)
    orientation_w = models.FloatField(default=0)


class Scene (models.Model):
    # Name of the scene, which fully identifies the Scene instance
    name = models.CharField(max_length=20, primary_key=True)

    # Mesh repository URL, relative to STATIC_URL
    mesh_url = models.URLField()

    # If the pose is null, the scene is empty
    pose = models.ForeignKey(Pose, null=True)
