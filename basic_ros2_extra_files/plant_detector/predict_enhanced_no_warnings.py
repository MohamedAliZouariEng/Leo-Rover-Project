import torch
import torch.nn as nn
from torchvision import transforms, models
from PIL import Image

# Load the trained model using the recommended method
model = models.resnet18(weights=None)  # Equivalent to pretrained=False
model.fc = nn.Sequential(
    nn.Linear(model.fc.in_features, 512),
    nn.ReLU(inplace=True),
    nn.Dropout(0.5),
    nn.Linear(512, 1)
)

# Load the saved state_dict with weights_only=True to avoid the security warning
model.load_state_dict(torch.load('best_plant_detector_model.pth', map_location=torch.device('cpu'), weights_only=True))
model.eval()

# Define image preprocessing
transform = transforms.Compose([
    transforms.Resize((150, 150)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

# Load and preprocess the image
img_path_yes = '/home/user/ros2_ws/src/plant_detector/tropical_plant_test.png'  # Replace with your image path
img_path_no = '/home/user/ros2_ws/src/plant_detector/no_tropical_plant.png'  # Replace with your image path
img_path_no2 = '/home/user/ros2_ws/src/plant_detector/no_tropical_plant2.png'  # Replace with your image path


image_test_array = [img_path_yes, img_path_no, img_path_no2]

for img_path in image_test_array:
    img = Image.open(img_path)
    # Convert the image to RGB if it is not
    if img.mode != 'RGB':
        img = img.convert('RGB')

    img = transform(img).unsqueeze(0)  # Add batch dimension

    # Predict if the image contains a plant
    with torch.no_grad():
        output = model(img)
        prediction = torch.sigmoid(output).item()

    if prediction > 0.5:
        print(f"The image contains a plant (Confidence: {prediction:.2f}).")
    else:
        print(f"The image does not contain a plant (Confidence: {1 - prediction:.2f}).")

