import torch, os
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt
from torchviz import make_dot

class GI_NN(nn.Module):
    def __init__(self, output_channels):
        super(GI_NN, self).__init__()
        self.gnn = nn.GRU(7, 128, 30)
        self.first_layer = nn.Conv1d(9, 1, 1)
        self.fc = nn.Linear(128, 64)
        self.drop_out = nn.Dropout(0.25)
        self.last_layer = nn.Linear(64, output_channels)

    def forward(self, x):
        x_list = []
        for i in range(x.shape[-1]):
            x_list.append(self.first_layer(torch.unsqueeze(x[:,:,i], dim=2)))
        a = torch.cat(tuple(x_list), 2)
        a = torch.unsqueeze(torch.sum(a, 1), 0)
        b = self.gnn(a)
        c = self.fc(b[0])
        d = self.drop_out(c)
        z = self.last_layer(d)
        z = torch.squeeze(z[0], dim=0)
        return z.float()

if __name__ == '__main__':
    os.chdir("/home/scout/catkin_ws/src/gps_nav/GPS")
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = GI_NN(output_channels=2)
    model.to(DEVICE)

    x = torch.randn(1, 9, 7).to(DEVICE)
    out = model(x)
    dot = make_dot(out.mean(), params=dict(model.named_parameters()))
    dot.format ="png"
    dot.render("model_arch")

    print(out[0][0][1])
