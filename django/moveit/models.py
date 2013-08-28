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

    def to_dict(self):
        return {
            'position': {
                'x': self.position_x,
                'y': self.position_y,
                'z': self.position_z,
            },
            'orientation': {
                'x': self.orientation_x,
                'y': self.orientation_y,
                'z': self.orientation_z,
                'w': self.orientation_w,
            }
        }


class Scene (models.Model):
    # Name of the scene, which fully identifies the Scene instance
    name = models.CharField(max_length=20, primary_key=True)

    # Mesh repository URL, relative to STATIC_URL
    mesh_url = models.URLField()

    # Thumbnail URL to show to users, relative to STATIC_URL
    thumbnail_url = models.URLField()

    # If the pose is null, the scene is empty
    pose = models.ForeignKey(Pose, null=True)

    def to_dict(self):
        pose = None
        if self.pose:
            pose = self.pose.to_dict()
        return {
            'name': self.name,
            'meshUrl': self.mesh_url,
            'pose': pose,
        }
