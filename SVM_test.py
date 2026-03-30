import sklearn
from sklearn import svm
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.metrics import roc_curve, auc, confusion_matrix, accuracy_score, f1_score
import os
import pandas as pd
import cv2
from sklearn.preprocessing import StandardScaler

flight_log = pd.read_csv('AE4317_2019_datasets/cyberzoo_poles_panels_mats/20190121-142943.csv')
poles = pd.read_csv('AE4317_2019_datasets/cyberzoo_poles_panels_mats/pole_locations.csv')
panels = pd.read_csv('AE4317_2019_datasets/cyberzoo_poles_panels_mats/panel_locations.csv')

obstacles_x = np.concatenate([poles['x'].values, panels['x'].values])
obstacles_y = np.concatenate([poles['y'].values, panels['y'].values])

def is_obstacle_in_view(drone_x, drone_y, drone_psi, max_dist=2.5, fov_angle=np.pi/4):
    """
    Checks if any obstacle is within 'max_dist' meters and inside the camera's FOV
    """
    dx = obstacles_x - drone_x
    dy = obstacles_y - drone_y
    distances = np.sqrt(dx**2 + dy**2)
    angles_to_obs = np.arctan2(dy, dx)
    angle_diffs = np.abs((angles_to_obs - drone_psi + np.pi) % (2*np.pi) - np.pi)
    threats = (distances < max_dist) & (angle_diffs < fov_angle)
    return 1 if threats.any() else 0

data_folder = 'AE4317_2019_datasets/cyberzoo_poles_panels_mats/20190121-142935/'
image_files = sorted(os.listdir(data_folder))

X_list = []
y_list = []
previous_gray = None

for filename in image_files:
    if not filename.endswith('.jpg'): continue
        
    img_time_sec = float(filename.replace('.jpg', '')) / 1000000.0
    
    time_diffs = np.abs(flight_log['time'] - img_time_sec)
    closest_idx = time_diffs.argmin()
    telemetry = flight_log.iloc[closest_idx]
    
    label = is_obstacle_in_view(
        drone_x=telemetry['pos_x'], 
        drone_y=telemetry['pos_y'], 
        drone_psi=telemetry['att_psi']
    )
    y_list.append(label)
    
    img_path = os.path.join(data_folder, filename)
    frame = cv2.imread(img_path)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    if previous_gray is not None:
        flow = cv2.calcOpticalFlowFarneback(previous_gray, gray, None, 0.5, 2, 11, 2, 5, 1.2, 0)
        
        u = flow[..., 0]
        v = flow[..., 1]
        
        # FEATURE 1: Overall Flow Magnitude
        flow_magnitude = np.mean(np.sqrt(u**2 + v**2))
        
        # FEATURE 2: True Divergence (Expansion) in the center of the camera
        du_dx = np.gradient(u, axis=1)
        dv_dy = np.gradient(v, axis=0)
        divergence_field = du_dx + dv_dy
        
        h, w = divergence_field.shape
        center_div = divergence_field[h//4 : 3*h//4, w//4 : 3*w//4]
        mean_divergence = np.mean(center_div)
        
        # FEATURE 3: Vertical Variance
        flow_y_var = np.var(v)
        
        X_list.append([flow_magnitude, mean_divergence, flow_y_var])
    else:
        X_list.append([0.0, 0.0, 0.0])
        
    previous_gray = gray

X = np.array(X_list)
y = np.array(y_list)

print(f"Dataset created: X shape {X.shape}, y shape {y.shape}")
print(f"Found {np.sum(y)} obstacle frames and {len(y) - np.sum(y)} safe frames.")


X_clean = X[1:]
y_clean = y[1:]

X_train, X_test, y_train, y_test = train_test_split(X_clean, y_clean, test_size=0.2, random_state=42)

clf = svm.SVC(kernel='linear',C=1.0, class_weight='balanced')

scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

clf.fit(X_train_scaled, y_train)

y_scores = clf.decision_function(X_test_scaled)
y_pred = clf.predict(X_test_scaled)

fpr, tpr, thresholds = roc_curve(y_test, y_scores)
roc_auc = auc(fpr, tpr)

plt.figure(figsize=(8, 6))
plt.plot(fpr, tpr, label=f'SVM ROC curve (AUC = {roc_auc:.2f})')
plt.xlabel('False Positive Rate (False Alarms)')
plt.ylabel('True Positive Rate (Successful Detections)')
plt.title('Receiver Operating Characteristic - Drone Obstacle Avoidance')
plt.legend(loc="lower right")
plt.grid(True)
plt.show()

print('confusion matrix:', confusion_matrix(y_test, y_pred))
print('accuracy:', accuracy_score(y_test, y_pred))
print('f1 score:', f1_score(y_test, y_pred))
