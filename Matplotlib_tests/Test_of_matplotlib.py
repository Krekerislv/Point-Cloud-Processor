import sys
import os
import matplotlib.pyplot as plt
import pandas as pd

if __name__ == "__main__":
    if (len(sys.argv) == 2):
        csv_path = sys.argv[1]
    else:
        print("Nav norādīts pareizs daudzums argumentu")
        sys.exit()

    if (not os.path.isfile(csv_path)):
        print("Fails neeksistē")
        sys.exit()

    #Nolasa failu
    points = 0 #mainīgais satur punktu x,y,z
    try:
        points = pd.read_csv(csv_path, sep=',')
        points = points.to_numpy()
    except:
        print("Radās kļūda. Vai norādīji pareizo failu?")
        sys.exit()

    print("CSV fails nolasīts, uzsāk vizualizāciju...")
    #izmantojot matplotlib, vizualizē mākoni
    size = 0.001 #katra punkta izmērs
    color = "b" #punktu krāsa
    figure = plt.figure()
    field = figure.add_subplot(projection='3d')
    field.scatter(points[:,0], points[:,1], points[:,2], s=size, c=color)
    field.view_init(elev=-165, azim=-45) #uzstāda sākuma kameras pozīciju
    plt.gca().set_aspect("equal")
    plt.title(os.path.basename(csv_path)) #mākoņa nosaukums = faila nosaukums
    plt.show()