import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from torchvision import transforms
from sklearn.model_selection import train_test_split
from sklearn.metrics import confusion_matrix, ConfusionMatrixDisplay
import matplotlib.pyplot as plt
import numpy as np
import os
import cv2
import random

# Set device (GPU if available, else CPU)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

path_data = os.path.join(".", "cropped")

CATEGORIES = ["up", "down", "left", "right"]
IMG_SIZE = 200

# Create training data set
training = []

def createTrainingData():
    count = 0
    for img in os.listdir(path_data):
        img_array = cv2.imread(os.path.join(path_data, img))
        new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))

        plt.figure()
        fig, ax = plt.subplots(2,1)
        ax[0].imshow(img_array)
        print('img_array')
        ax[1].imshow(new_array)
        print('new_array')

        plt.show()

        # label
        category = img.split("_", 1)[0]
        class_num = CATEGORIES.index(category)
        training.append([new_array, class_num])

        if count > 3:
            break
        count+=1

createTrainingData()

random.shuffle(training)
print(len(training))
print(len(training[0]))
print(len(training[0][0]))
print(len(training[0][0][0]))
print(len(training[0][0][0][0]))

# Assigning Labels and Features
X = []
y = []
count = 0
for features, label in training:
    print(features.shape)
    print(features.transpose(2,0,1).shape)
    X.append(features.transpose(2,0,1))
    y.append(label)
    print(len(X[0]))
    if count > 3:
        break
    count+=1
X = np.array(X)
print(X.shape)
X = X.reshape(-1, IMG_SIZE, IMG_SIZE, 3)
print(X.shape)

# # Normalize X and Convert Labels to Categorical Data
# X = X.astype('float32') / 255.0
# Y = torch.tensor(y, dtype=torch.long, device=device)
# Y_onehot = torch.eye(len(CATEGORIES), device=device)[Y]

# # Split X and Y for Use in CNN
# X_train, X_test, y_train, y_test = train_test_split(X, Y_onehot.cpu().numpy(), test_size=0.2, random_state=4)

# # Convert to PyTorch tensors
# X_train_tensor = torch.tensor(X_train.transpose((0,3,1,2)), dtype=torch.float32, device=device)
# y_train_tensor = torch.tensor(y_train, dtype=torch.long, device=device)
# X_test_tensor = torch.tensor(X_test.transpose((0,3,1,2)), dtype=torch.float32, device=device)
# y_test_tensor = torch.tensor(y_test, dtype=torch.long, device=device)

# Define and Train the CNN Model in PyTorch
class CNNModel(nn.Module):
    def __init__(self):
        super(CNNModel, self).__init__()
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, padding=1)
        self.relu1 = nn.ReLU()
        self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.conv2 = nn.Conv2d(32, 32, kernel_size=3, padding=1)
        self.relu2 = nn.ReLU()
        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(32 * (IMG_SIZE // 4) * (IMG_SIZE // 4), 128)
        self.relu3 = nn.ReLU()
        self.dropout = nn.Dropout(0.5)
        self.fc2 = nn.Linear(128, len(CATEGORIES))

    def forward(self, x):
        x = self.conv1(x)
        x = self.relu1(x)
        x = self.pool1(x)
        x = self.conv2(x)
        x = self.relu2(x)
        x = self.pool2(x)
        x = self.flatten(x)
        x = self.fc1(x)
        x = self.relu3(x)
        x = self.dropout(x)
        x = self.fc2(x)
        return x

# model = CNNModel().to(device)

# # Define loss function and optimizer
# criterion = nn.CrossEntropyLoss()
# optimizer = optim.Adam(model.parameters(), lr=0.001)

# # Training loop
# batch_size = 32
# nb_epochs = 50

# for epoch in range(nb_epochs):
#     model.train()

#     for i in range(0, len(X_train_tensor), batch_size):
#         inputs = X_train_tensor[i:i+batch_size]
#         labels = y_train_tensor[i:i+batch_size]

#         optimizer.zero_grad()

#         outputs = model(inputs)
#         loss = criterion(outputs, torch.argmax(labels, dim=1))
#         loss.backward()
#         optimizer.step()
    
#     print(f"Epoch : {epoch}")
#     print(f"Loss : {loss}")

# # Evaluation
# model.eval()
# with torch.no_grad():
#     outputs = model(X_test_tensor)
#     _, predicted = torch.max(outputs, 1)
#     y_test = np.argmax(y_test_tensor.cpu().numpy(), axis=1)

# # Confusion matrix
# result = confusion_matrix(y_test, predicted.cpu().numpy())
# cm = ConfusionMatrixDisplay(result, display_labels=['Up', 'Down', 'Left', 'Right'])
# cm.plot()
# plt.show()

# # Save the model
# torch.save(model.state_dict(), 'cnn_model.pth')