import torch

class PeopleDetection:

    def __init__(self):
        self.predictions = []
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', '/home/guilherme/Documents/py/testing_hardtech/best.pt')  # custom trained model
        self.model.conf = 0.65

    def detection(self, cap = 0):

        #ret, img = cap.read()  # or file, Path, PIL, OpenCV, numpy, list
        img = '/home/guilherme/Documents/py/testing_hardtech/images6.jpg'
        results = self.model(img)
        results.save(save_dir='results')
        print(self.predictions)
        print(results)



def main():

    pd = PeopleDetection()

    pd.detection()

    # while(True):
    #     try:
    #         detec
    #     except:
    #         break