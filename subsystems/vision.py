import photonvision

class Vision:
    def __init__(self, config):
        self.visionCamera = photonvision.PhotonCamera("photonvision")

    def getTargetCorners(self):
        result = self.visionCamera.getLatestResult()
        if result.hasTargets():
            corners = result.getBestTarget().getCorners()
        return(corners)