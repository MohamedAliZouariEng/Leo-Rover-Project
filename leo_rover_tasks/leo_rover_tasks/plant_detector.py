import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image as PILImage

class PlantDetector:
    def __init__(self, model_path='best_plant_detector_model.pth'):
        # Load the trained model
        self.model = models.resnet18(weights=None)
        self.model.fc = nn.Sequential(
            nn.Linear(self.model.fc.in_features, 512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.5),
            nn.Linear(512, 1)
        )
        self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
        self.model.eval()
        
        # Define image preprocessing
        self.transform = transforms.Compose([
            transforms.Resize((150, 150)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

    def predict(self, image):
        # Convert input image to PIL Image if it's not already
        if not isinstance(image, PILImage.Image):
            image = PILImage.fromarray(image)
        
        # Preprocess the image
        input_tensor = self.transform(image).unsqueeze(0)
        
        # Run the model to detect a plant
        with torch.no_grad():
            output = self.model(input_tensor)
            prediction = torch.sigmoid(output).item()
        
        return prediction
