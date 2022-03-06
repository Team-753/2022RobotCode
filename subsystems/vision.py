import photonvision

class Vision:
    def __init__(self, config):
        self.visionCamera = photonvision.PhotonCamera(config["Camera"]["Name"])
        self.cameraHeight = config["Camera"]["cameraHeight"]
        self.targetHeight = config["Camera"]["targetHeight"]

    def getTargetCorners(self):
        result = self.visionCamera.getLatestResult()
        if result.hasTargets():
            corners = result.getBestTarget().getCorners()
        return(corners)
    
    def searchAndDestroy(self):
        pass
    
    def visionOff(self):
        pass