import torch, os
import torch.nn as nn
import torch.nn.functional as F
from torch.optim import SGD
import matplotlib.pyplot as plt
from GI_NN import GI_NN
from dataloader import IMUDataset
import wandb, datetime
import numpy as np

train_path = "data/data.txt"
val_path = "data/val.txt"

def normalize_data(X):
    x = np.asanyarray(X)
    print(x.shape)
    return (x - np.min(x, axis=0)) / (np.max(x, axis=0) - np.min(x, axis=0))

def extract_txt(file_path):
    points = []
    X, y = [], []

    with open(file_path, 'r') as f:
        line = f.readlines()
        f.close()
    for l in line:
        pt = list(map(np.float64, l.split(",")))
        points.append(pt)
    for pt in points:
        X.append(pt[2:])
        y.append(pt[:2])

    return X, y

def train_one_epoch():
    running_loss = 0.
    last_loss = 0.
    for i, data in enumerate(training_loader):
        try:
            X, y = data
            X = X.float()
            y = y.float()
            optimizer.zero_grad()
            y_ = model(X)
            loss = loss_fn(y_, y)
            loss.backward()

            optimizer.step()

            running_loss += loss.item()
            if i%10 == 9:
                last_loss = running_loss / 1000
                print(f"Batch {i+1} loss: {last_loss}")
                running_loss = 0
        except:
            print("[INFO] Not enough data, proceeding...")
            break

    return last_loss

Xt, yt = extract_txt(train_path)
Xv, yv = extract_txt(val_path)

Xt = normalize_data(Xt)
Xv = normalize_data(Xv)

print(Xt)

training_loader = IMUDataset(Xt, yt, seq_len=7)
validation_loader = IMUDataset(Xv, yv, seq_len=7)

model = GI_NN(output_channels=2)
model = model.float()
model.train()
optimizer = SGD(model.parameters(), lr = 0.0003, momentum=0.9)
loss_fn = nn.MSELoss()

if __name__ == '__main__':
    # wandb.init(project="GI_NN")
    preds, labels = [], []
    EPOCH = 15
    train_loss, val_loss = [], []
    train_loss_all, val_loss_all = [], []
    epoch_number = 0
    time_stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    best_vloss = 99999
    for epoch in range(EPOCH):
        labels = []
        preds = []
        avg_loss = train_one_epoch()
        print("Epoch Done")
        running_vloss = 0.
        with torch.no_grad():
            for i, vdata in enumerate(validation_loader):
                try:
                    vX, vy = vdata
                    vX = vX.float()
                    vy = vy.float()
                    vy_ = model(vX)
                    labels.append(vy.tolist())
                    preds.append(vy_.tolist())
                    vloss = loss_fn(vy_, vy)
                    running_vloss += vloss
                except:
                    print("[INFO] Not enough data, proceeding...")
                    break
        avg_vloss = running_vloss / (i+1)
        print(f"Loss train {avg_loss}, validation {avg_vloss}")
        train_loss.append(avg_loss)
        val_loss.append(avg_vloss)

        # wandb.log({
        #     "Training": avg_loss,
        #     "Validation": avg_vloss,
        #     "Epoch":epoch_number
        # })
    
        if avg_vloss < best_vloss:
            best_vloss = avg_vloss
            model_path = f"chkpts/model_{time_stamp}_{epoch_number}"
            torch.save(model.state_dict(), model_path)
        
        epoch_number += 1
        model.train()
    
    px, py = [], []
    for idx, p in enumerate(preds):
        if idx == 0:
            px.append(p[0])
            py.append(p[1])
        else:
            px.append(px[idx-1] + p[0])
            py.append(py[idx-1] + p[1])
    
    lx, ly = [], []
    for idx, l in enumerate(labels):
        if idx == 0:
            lx.append(l[0])
            ly.append(l[1])
        else:
            lx.append(lx[idx-1] + l[0])
            ly.append(ly[idx-1] + l[1])

    xx = 0
    yy = 0
    for p in preds:
        xx += l[0]
        yy += l[1]

    print(xx, yy)

    plt.plot(train_loss)
    plt.title("Training Loss")
    plt.xlabel("Epochs")
    plt.ylabel("Loss")
    plt.show()

    plt.plot(val_loss)
    plt.title("Validation Loss")
    plt.xlabel("Epochs")
    plt.ylabel("Loss")
    plt.show()

    plt.plot(px, py)
    plt.show()

    plt.plot(lx, ly)
    plt.show()
