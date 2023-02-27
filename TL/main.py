
import pandas as pd
import numpy as np

# for instance 1:
# x_max_per_truck = [18,17,18,17,18,17,18,17,18,17,18,17,20,18,17,18,17,18,17,35,30,18,17,20,18,17,20,18,17,20]

# for instance 2:
x_max_per_truck = [20, 35, 30, 18, 17, 18, 17, 18, 17, 20, 18, 17, 20, 18, 17, 18, 17, 35, 30, 20, 18, 17, 18, 17, 20, 35, 30, 18, 17, 18, 17, 20]
len(x_max_per_truck)
names = ["truck","route",'configuration', 'level','cornerX', 'cornerY','lengthX', 'lengthY', "max_length"]
configs_df = pd.read_csv(r'C:\Users\JP\IdeaProjects\TL\config_matrix.csv', header= None)

configs_df.columns = names
configs_df = configs_df.loc[~(configs_df==0).all(axis=1)]


for i in range(len(configs_df)):
    configs_df.iloc[i]['max_length'] = x_max_per_truck[int(configs_df.iloc[i]['truck'])]*(configs_df.iloc[i]['level']+1)

counter = 0
for i in range(len(configs_df)):
    if configs_df.iloc[i]['cornerX'] + configs_df.iloc[i]['lengthX'] > configs_df.iloc[i]['max_length']:
        print(i, configs_df.iloc[i]['cornerX'] + configs_df.iloc[i]['lengthX'] - configs_df.iloc[i]['max_length'])
        # print(i)
        counter += 1

counter


len(configs_df)


x_max_per_truck[17]
print(configs_df.iloc[213433])



configs = pd.DataFrame(df).to_numpy()

new_col = np.zeros(len(configs))
new_col = new_col.reshape((len(configs),1))

configs = np.hstack((configs, new_col))

for i in range(len(configs)):
    for j in range(len(x_max_per_truck)):
        if configs[i, 0] == j:
            if configs[i, 3] == 0:
                configs[i, 8] = x_max_per_truck[j]
            else:
                configs[i,8] = x_max_per_truck[j]*2


configs_df = pd.DataFrame(configs, columns=names)


for i in range(len(configs_df)):
    if configs_df.iloc[i]['cornerX'] + configs_df.iloc[i]['lengthX'] > configs_df.iloc[i]['max_length']:
        print(i)
        # print(configs_df.iloc[i])


































