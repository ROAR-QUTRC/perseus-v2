import pandas as pd
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
# channel definitions
channels = [
    ("FZ_450", 450),
    ("FY_555_wide", 555),
    ("FXL_600", 600),
    ("NIR_855", 855),
    ("VIS1", None),
    ("FD1", None),
    ("F2_425", 425),
    ("F3_475", 475),
    ("F4_515", 515),
    ("F6_640", 640),
    ("VIS2", None),
    ("FD2", None),
    ("F1_405", 405),
    ("F7_690", 690),
    ("F8_745", 745),
    ("F5_550_narrow", 550),
    ("VIS3", None),
    ("FD3", None),
]


# channels we care about
useful_channels = [
    "F1_405", "F2_425", "FZ_450",
    "F3_475", "F4_515",
    "F5_550_narrow", "FY_555_wide",
    "FXL_600", "F6_640",
    "F7_690", "F8_745",
    "NIR_855",
]

# loading in data
samples = {
    2:  "ilmenite_2.csv",
    5:  "ilmenite_5.csv",
    10: "ilmenite_10.csv",
    15: "ilmenite_15.csv",
}

dfs = []

channel_names = [c[0] for c in channels]

for wt, fname in samples.items():
    df = pd.read_csv(fname, header=None)
    df.columns = channel_names
    df["label"] = wt
    dfs.append(df)

data = pd.concat(dfs, ignore_index=True)

EPS = 1e-6

data["Ratio_F1_F5"] = data["F1_405"] / (data["F5_550_narrow"] + EPS)
data["Ratio_F2_F7"] = data["F2_425"] / (data["F7_690"] + EPS)
data["Ratio_F4_F6"] = data["F4_515"] / (data["F6_640"] + EPS)
data["Ratio_F2_F6"] = data["F2_425"] / (data["F6_640"] + EPS)
data["Ratio_F2_F5"] = data["F2_425"] / (data["F5_550_narrow"] + EPS)
data["Ratio_F5_F7"] = data["F5_550_narrow"] / (data["F7_690"] + EPS)
data["Ratio_F3_F7"] = data["F3_475"] / (data["F7_690"] + EPS)



#supplying knn with ratios instead of magnituedes
feature_cols = [
    "Ratio_F1_F5",
    "Ratio_F2_F7",
    "Ratio_F4_F6",
    "Ratio_F2_F6",
    "Ratio_F2_F5",
    "Ratio_F5_F7",
    "Ratio_F3_F7",
]

X = data[feature_cols].values
y = data["label"].values



# scaling data so x axis doesn't affect neighbours classification
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

#training the classifier
#k-value should always be odd so it won't have to choose between two classifications (if result is even)
knn = KNeighborsClassifier(
    n_neighbors=5, #k-value of 5, can be changed depending on accuracy
    weights="distance",
    metric="euclidean"
)

knn.fit(X_scaled, y) #storing data so it doesn't have to calcualte again


#evaluating the model
#testing 25%, training 75%
X_train, X_test, y_train, y_test = train_test_split( X_scaled, y, test_size=0.6, stratify=y, random_state=42) 

#to help choose best k value
for k in range(1, 21):
    knn = KNeighborsClassifier(n_neighbors=k, weights="distance")
    knn.fit(X_train, y_train)
    acc = knn.score(X_test, y_test)
    print(k, acc)

#k value of 3-9 have the best accuracy of 0.8333

knn.fit(X_train, y_train) #storing data so it doesn't have to calculate again
y_pred = knn.predict(X_test)

print(confusion_matrix(y_test, y_pred))
print(classification_report(y_test, y_pred))

#in classification report
#precision tell us how often the model is correct when predicting this class
#recall tell us how many true samples it found
#support is how many test samples exist for this class

#apparently 2% gets confused with other concentrations (identified all 2% samples but also identified others wrongly as 2%)
#everytime it guessed 5%, it was correct, however it fails to guess all 5% samples. So 5% is getting confused with 2% and 10%
#identified every 10% sample but also wrongly identified some samples as 10%
#identified every 15% sample and did not wrongly identify any samples as 15%

new_measurement = {
    "F1_405": 12,
    "F2_425": 48,
    "F3_475": 54,
    "F4_515": 62,
    "F5_550_narrow": 88,
    "F6_640": 70,
    "F7_690": 32,
}

# compute ratios
x_new = np.array([[
    new_measurement["F1_405"] / (new_measurement["F5_550_narrow"] + EPS),
    new_measurement["F2_425"] / (new_measurement["F7_690"] + EPS),
    new_measurement["F4_515"] / (new_measurement["F6_640"] + EPS),
    new_measurement["F2_425"] / (new_measurement["F6_640"] + EPS),
    new_measurement["F2_425"] / (new_measurement["F5_550_narrow"] + EPS),
    new_measurement["F5_550_narrow"] / (new_measurement["F7_690"] + EPS),
    new_measurement["F3_475"] / (new_measurement["F7_690"] + EPS),
]])

x_new_scaled = scaler.transform(x_new)

prediction = knn.predict(x_new_scaled)
print("Predicted ilmenite wt.% =", prediction[0])