""" Process visual features and affordance experiment results for BN training
This program read the visual features and affordance experiment results
and discretize the values.
The results are output to a CSV file.
"""
# Author: Yik Lung Pang

import json
import csv
import bisect
import random
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
import pandas as pd
import numpy as np
import os



def main():
    """Main function to process the visual features and affordanc experiment
    results

    """
    include_tool_features = ['cubeness', 'symmetry', 'convexity', 'eccentricity']
    include_obj_features = ['sphereness', 'cubeness', 'symmetry', 'eccentricity']
    preprocess_data('pc_features.csv', False, include_tool_features, include_obj_features, 'scaled_features.csv', 'train_data.csv', 'test_data.csv', 'loo_data.csv')
    preprocess_data('pc_features.csv', True, include_tool_features, include_obj_features, 'pca_features.csv', 'pca_train_data.csv', 'pca_test_data.csv', 'pca_loo_data.csv')
    include_tool_features = ['squareness', 'symmetry', 'convexity', 'eccentricity']
    include_obj_features = ['circleness']
    preprocess_data('rgb_features.csv', False, include_tool_features, include_obj_features, 'scaled_features_2d.csv', 'train_data_2d.csv', 'test_data_2d.csv', 'loo_data_2d.csv')
    preprocess_data('rgb_features.csv', True, include_tool_features, include_obj_features, 'pca_features_2d.csv', 'pca_train_data_2d.csv', 'pca_test_data_2d.csv', 'pca_loo_data_2d.csv')

def preprocess_data(feature_file_name, perform_PCA, include_tool_features, include_obj_features, scaled_fname, train_output_fname, test_output_fname, loo_output_fname):
    """Process the visual features and affordanc experiment results

    Parameters
    ----------
    feature_file_name : str
        The list of tools and objects names
    perform_PCA : bool
        The list of tools and objects names
    include_tool_features : list of str
        The list of tools and objects names
    include_obj_features : list of str
        The list of tools and objects names
    scaled_fname : str
        File name to output scaled features
    train_output_fname : str
        File name to output training data
    test_output_fname : str
        File name to output test data
    loo_output_fname : str
        File name to output training data

    """

    # Read feature values into a dictionary
    script_dir = os.path.dirname(os.path.realpath(__file__))
    pc_obj_dict = {}
    pc_features_file_path = str(script_dir+'/features/'+feature_file_name)
    features = [] # feature names
    with open(pc_features_file_path, 'r') as pc_features_csv:
        read_csv = csv.reader(pc_features_csv, delimiter=',')
        header = True
        for row in read_csv:
            # Process header
            if header == True:
                header = False
                features = row[1:]
                print(features)
                continue

            # Process object line
            object_name = row[0]
            if object_name in pc_obj_dict:
                print('Warning: Duplicate column found for '+object_name)
                continue
            else:
                pc_obj_dict[object_name] = {}
                for idx, feature_name in enumerate(features):
                    pc_obj_dict[object_name][feature_name] = row[idx+1]

    pca_df = pd.DataFrame()
    pca_n_components = 2
    if perform_PCA:
        # For combinations of tools and objects, create PCA vectors
        column_names = ["tool_name", "object_name",
                  "tool_"+features[0], "tool_"+features[1], "tool_"+features[2], "tool_"+features[3], "tool_"+features[4],
                  "obj_"+features[0], "obj_"+features[1], "obj_"+features[2], "obj_"+features[3], "obj_"+features[4]]

        # Objects used for fitting PCA, exclude fork
        tools = ['stick', 'l_stick', 'bone', 'umbrella']
        objects = ['cube', 'sphere', 'cylinder']

        # Load training features
        train_df_data = []
        for tool in tools:
            for obj in objects:
                train_df_data.append([tool, obj,
                                pc_obj_dict[tool][features[0]], pc_obj_dict[tool][features[1]], pc_obj_dict[tool][features[2]], pc_obj_dict[tool][features[3]], pc_obj_dict[tool][features[4]],
                                pc_obj_dict[obj][features[0]], pc_obj_dict[obj][features[1]], pc_obj_dict[obj][features[2]], pc_obj_dict[obj][features[3]], pc_obj_dict[obj][features[4]]])

        train_df = pd.DataFrame(train_df_data, columns=column_names, dtype=float)

        # Load testing features
        test_df_data = []
        tools = ['fork']
        for tool in tools:
            for obj in objects:
                test_df_data.append([tool, obj,
                                pc_obj_dict[tool][features[0]], pc_obj_dict[tool][features[1]], pc_obj_dict[tool][features[2]], pc_obj_dict[tool][features[3]], pc_obj_dict[tool][features[4]],
                                pc_obj_dict[obj][features[0]], pc_obj_dict[obj][features[1]], pc_obj_dict[obj][features[2]], pc_obj_dict[obj][features[3]], pc_obj_dict[obj][features[4]]])

        test_df = pd.DataFrame(test_df_data, columns=column_names, dtype=float)

        # Feature names for fitting PCA
        pca_features = ["tool_"+features[0], "tool_"+features[1], "tool_"+features[2], "tool_"+features[3], "tool_"+features[4],
                        "obj_"+features[0], "obj_"+features[1], "obj_"+features[2], "obj_"+features[3], "obj_"+features[4]]

        # Extract feature values into dataframe
        train_x = train_df.loc[:, pca_features].values
        test_x  = test_df.loc[:, pca_features].values

        print(train_x)
        print(test_x)

        # Fit standard scaler on training features
        scaler = StandardScaler()
        scaler.fit(train_x)
        # Apply fitted standard scaler to training and testing features
        train_x = scaler.transform(train_x)
        test_x = scaler.transform(test_x)

        # Fit PCA on training features
        pca = PCA(n_components=pca_n_components, svd_solver = 'full')
        pca.fit(train_x)
        print("PCA no. of components: "+str(pca.n_components_))
        print("PCA explained variance ratio: "+str(pca.explained_variance_ratio_))
        print("PCA explained variance ratio: "+str(sum(pca.explained_variance_ratio_)))
        # Apply fitted PCA to training and testing features
        train_x = pca.transform(train_x)
        test_x = pca.transform(test_x)

        # Combine dataframe of training and testing features
        pca_column_names = []
        for i in range(pca.n_components_):
            pca_column_names.append("principal_component_"+str(i))

        train_df_pca = pd.DataFrame(data = train_x, columns = pca_column_names)
        train_df_pca = pd.concat([train_df[["tool_name","object_name"]], train_df_pca], axis = 1)
        test_df_pca = pd.DataFrame(data = test_x, columns = pca_column_names)
        test_df_pca = pd.concat([test_df[["tool_name","object_name"]], test_df_pca], axis = 1)
        pca_df = pd.concat([train_df_pca, test_df_pca], axis = 0)
        
        # Output PCA features
        pca_file_path = os.path.join(script_dir, 'processed_data', scaled_fname)
        with open(pca_file_path, 'w') as pca_features_file:
            pca_features_file.write(pca_df.to_csv(index=False))
    else:
        # No PCA, just standardize feature values

        # Objects used for fitting standard scaler, exclude fork
        scaling_list = ['stick', 'l_stick', 'bone', 'umbrella', 'cube', 'sphere', 'cylinder']
        # Load training features
        scaling_train = []
        for item in scaling_list:
            scaling_train.append([item,pc_obj_dict[item][features[0]], pc_obj_dict[item][features[1]], pc_obj_dict[item][features[2]], pc_obj_dict[item][features[3]], pc_obj_dict[item][features[4]]]) 
        # Load testing features
        scaling_test = [['fork',pc_obj_dict['fork'][features[0]], pc_obj_dict['fork'][features[1]], pc_obj_dict['fork'][features[2]], pc_obj_dict['fork'][features[3]], pc_obj_dict['fork'][features[4]]]]  

        # Convert to dataframes
        column_names = ['item'] + features
        scaling_train_df = pd.DataFrame(scaling_train, columns=column_names, dtype=float)
        train_x = scaling_train_df.loc[:, features].values
        scaling_test_df = pd.DataFrame(scaling_test, columns=column_names, dtype=float)
        test_x = scaling_test_df.loc[:, features].values    

        # Fit standard scaler on training features
        scaler = StandardScaler()
        scaler.fit(train_x)
        # Apply fitted standard scaler to training and testing features
        train_x = scaler.transform(train_x)
        test_x = scaler.transform(test_x)   

        # Replace original feature values with standardised values
        scaled_data = np.concatenate((train_x,test_x))
        scaling_list = scaling_list + ['fork']
        for i in range(scaled_data.shape[0]):
            for j in range(scaled_data.shape[1]):
                pc_obj_dict[scaling_list[i]][features[j]] = scaled_data[i][j]

        # Convert standardised values to discrete labels
        scaled_range = [-1.0, 0.0, 1.0]
        pc_labels = ['L', 'ML', 'MH', 'H']
        for object_name in pc_obj_dict:
            for feature_name, value in pc_obj_dict[object_name].items():
                pos = bisect.bisect(scaled_range, float(value))
                pc_obj_dict[object_name][feature_name] = pc_labels[pos]

        scaled_data_df = pd.DataFrame(scaled_data, columns=features, dtype=float)
        scaled_file_path = os.path.join(script_dir, 'processed_data', scaled_fname)
        with open(scaled_file_path, 'w') as scaled_file:
            scaled_file.write(scaled_data_df.to_csv(index=False))

    # Discretization for action
    x_range = [-0.08,-0.04,0.00,0.04,0.08]
    y_range = [-0.08,-0.04,0.00,0.04,0.08]
    action_labels = ['VN','LN','NM','NM','LP','VP']

    # Process affordance experiment data
    # First 80% goes to training, remaining 20% goes to testing
    # This ensures the training set has even distribution of each object, tool and action
    # while achieveing randomisation (experiments are independent from each other)
    train_list = []
    test_list = []
    loo_list = []
    # Open data directory
    experiment_data_directory_path = str(script_dir+'/data')
    for filename in os.listdir(experiment_data_directory_path):
        print(filename+" processed.")
        experiment_data_file_path = os.path.join(experiment_data_directory_path, filename)
        # Go through results for all tools
        with open(experiment_data_file_path, 'r') as experiment_data_json:
            experiment_data = json.load(experiment_data_json)
            tool_name = experiment_data['tools']

            # Go through results for all objects
            for object_name in experiment_data['objects']:
                result_cnt = 0
                for result in experiment_data['objects'][object_name]['results']:
                    result_cnt += 1
                    # Object bounced off work table, discard
                    if result['end_pos']['z'] < 0.5:
                        print("WARNING: Invalid result. Discarded")
                        print(tool_name, object_name, action)
                        continue

                    action = result['action']
                    dx = result['end_pos']['x'] - result['start_pos']['x']
                    dy = result['end_pos']['y'] - result['start_pos']['y']
                    effect_x = action_labels[bisect.bisect(x_range, dx)]
                    effect_y = action_labels[bisect.bisect(y_range, dy)]

                    # Construct row for this experiment
                    temp_row = []
                    # Populate features
                    if perform_PCA:
                        pca_list = findPCA(tool_name, object_name, pca_df, pca_n_components)
                        for pca_value in pca_list:
                            temp_row.append(pca_value)
                    else:
                        for f in include_tool_features:
                            temp_row.append(pc_obj_dict[tool_name][f])
                        for f in include_obj_features:
                            temp_row.append(pc_obj_dict[object_name][f])
                    # Populate action and effects
                    temp_row.append(action)
                    temp_row.append(effect_x)
                    temp_row.append(effect_y)

                    # Append to data list
                    if tool_name == 'fork':
                        loo_list.append(temp_row)
                    elif result_cnt <= 32:
                        train_list.append(temp_row)
                    else:
                        test_list.append(temp_row)

    # Output to CSV files
    # Construct header
    csv_header = []
    if perform_PCA:
        for i in range(pca_n_components):
            csv_header.append('principal_component_'+str(i))
    else:
        for f in include_tool_features:
            csv_header.append("tool_"+f)
        for f in include_obj_features:
            csv_header.append("obj_"+f)
    csv_header.append("action")
    csv_header.append("effect_X")
    csv_header.append("effect_Y")

    # Output training data
    train_cnt = 0
    train_data_file_path = str(script_dir+'/processed_data/'+train_output_fname)
    with open(train_data_file_path, 'w') as train_data_file:
        data_writer = csv.writer(train_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
        data_writer.writerow(csv_header)
        for i in range(len(train_list)):
            data_writer.writerow(train_list[i])
            train_cnt += 1
    print("Train file created with "+str(train_cnt)+" entries.")

    # Output test data
    test_cnt = 0
    test_data_file_path = str(script_dir+'/processed_data/'+test_output_fname)
    with open(test_data_file_path, 'w') as test_data_file:
        data_writer = csv.writer(test_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
        data_writer.writerow(csv_header)
        for i in range(len(test_list)):
            data_writer.writerow(test_list[i])
            test_cnt += 1
    print("Test file created with "+str(test_cnt)+" entries.")

    # Output leave one out test data
    loo_cnt = 0
    loo_data_file_path = str(script_dir+'/processed_data/'+loo_output_fname)
    with open(loo_data_file_path, 'w') as loo_data_file:
        data_writer = csv.writer(loo_data_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
        data_writer.writerow(csv_header)
        for i in range(len(loo_list)):
            data_writer.writerow(loo_list[i])
            loo_cnt += 1
    print("Leave one out test file created with "+str(loo_cnt)+" entries.")

def findPCA(tool, obj, df, n_components):
    """Find the PCA values for specific tool and object

    Parameters
    ----------
    tool : str
        tool name
    obj : bool
        objects name
    df : list of str
        dataframe storing the PCA values
    n_components : list of str
        number of PCA components


    """
    ranges = [-1.75, -1.0, 0.0, 1.0, 1.75]
    labels = ['VL', 'L', 'ML', 'MH', 'H', 'VH']
    pca_label_list = []
    for index, row in df.iterrows():
        if row['tool_name'] == tool and row['object_name'] == obj:
            for i in range(n_components):
                pca_label_list.append(labels[bisect.bisect(ranges, row['principal_component_'+str(i)])])
            return pca_label_list


## Main function
if __name__ == "__main__":
    main()