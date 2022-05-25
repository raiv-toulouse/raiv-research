import os
import torch
import shutil
from raiv_libraries.image_model import ImageModel
from raiv_libraries.image_tools import ImageTools
from PIL import Image


FAIL=0
SUCCESS=1


class TestPred:
    def __init__(self, ckpt_file, image_dir, destination):
        self.image_dir = image_dir
        self.destination = destination
        self.image_model = ImageModel(model_name='resnet18', ckpt_dir=os.path.dirname(ckpt_file))
        self.inference_model = self.image_model.load_ckpt_model_file(os.path.basename(ckpt_file))
        files = self.load_files(image_dir+'/fail')
        self.predict_image(files, image_dir, FAIL, destination)
        files = self.load_files(image_dir+'/success')
        self.predict_image(files, image_dir, SUCCESS, destination)

    def load_files(self, image_dir):
        files = []
        for entry in os.walk(image_dir):
            files.append(entry)
        files = files[0][2]
        return files

    def predict_image(self, files, image_dir, type_recherche, destination):
        for i in range(len(files)):
            if type_recherche == FAIL :
                chemin = image_dir + '/fail/' + str(files[i])
            if type_recherche == SUCCESS :
                chemin = image_dir + '/success/' + str(files[i])

            self.image = Image.open(chemin)
            img = ImageTools.transform_image(self.image)  # Get the loaded images, resize in 256 and transformed in tensor
            img = img.unsqueeze(0)  # To have a 4-dim tensor ([nb_of_images, channels, w, h])
            features, preds = self.image_model.evaluate_image(img, False)  # No processing
            proba_success = torch.exp(preds[0][1]).item()
            proba_success = round(proba_success, 3)

            if type_recherche == FAIL and proba_success > 0.5:
                destination2 = '/home' + destination + '/false_fail/proba_' + str(proba_success) + '.png'
                shutil.copy(chemin, destination2)

            if type_recherche == SUCCESS and proba_success <= 0.5:
                destination2 = '/home' + destination + '/false_success/proba_' + str(proba_success) + '.png'
                shutil.copy(chemin, destination2)

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Test an image prediction and write the false predictions in a directory with the pourcentage')
    parser.add_argument('ckpt_file', type=str, help='model .ckpt')
    parser.add_argument('rep_image', type=str, help='directory with the image bank')
    parser.add_argument('rep_arrive', type=str, help='directory where the images are placed')
    args = parser.parse_args()

    test_pred = TestPred(ckpt_file=args.ckpt_file, image_dir=args.rep_image, destination=args.rep_arrive)

