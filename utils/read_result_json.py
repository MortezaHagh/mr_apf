import os
import json
import pandas as pd

class JsonResults:
    def __init__(self):
        pass
    
    def get_json_value(self, file_path, key):
        with open(file_path) as f:
            data = json.load(f)

            # Check if data is a JSON object or array
            if isinstance(data, dict):
                return data.get(key)
            elif isinstance(data, list):
                for element in data:
                    if isinstance(element, dict):
                        value = element.get(key)
                        if value is not None:
                            return value

            # Key not found
            return None

    def get_one_value(self, file_path, key):
        file_path = "/home/morteza/Documents/Morteza/CurrentAPF/result_apf/res_5.json"
        key = "mean_len"
        value = self.get_json_value(file_path, key)
        print(f"The value of the key '{key}' is '{value}'")

    def create_table(self, directory, keys):
        
        # List to store values for each file
        file_values = []

        # Iterate over files in directory
        for filename in os.listdir(directory):
            if filename.endswith(".json"):
                a = filename.split('_')
                t = int(a[1][:])
                a = filename.split('.')
                v = int(a[0][-1])
                file_path = os.path.join(directory, filename)

                # Retrieve values for each key in the list
                values = [self.get_json_value(file_path, key) for key in keys]
                values.insert(0, v)
                values.insert(0, t)

                # Add the values to the file_values list
                file_values.append(values)

        # Create a Pandas DataFrame using the retrieved values
        keys.insert(0, 'version')
        keys.insert(0, 'test')
        df = pd.DataFrame(file_values, columns=keys)

        # Sort the DataFrame based on the values of the first key (i.e., "name")
        df = df.sort_values(by=keys[0])

        # Save the DataFrame as a CSV file
        df.to_csv(directory+"output.csv", index=False)

        # Print confirmation message
        print("CSV file saved successfully!")

        # Print the DataFrame
        print(df)

    def create_table_2(self, directory, keys):
        # List to store values for each file
        file_values = []

        # Iterate over files in directory
        for filename in os.listdir(directory):
            if filename.endswith(".json"):
                a = filename.split('_')
                t = int(a[1][:])
                a = filename.split('.')
                v = int(a[0][-1])
                file_path = os.path.join(directory, filename)

                # Retrieve values for each key in the list
                values = [self.get_json_value(file_path, key) for key in keys]
                values.insert(0, v)
                values.insert(0, t)
                
                # Add the values to the file_values list
                file_values.append(values)

        # Create a Pandas DataFrame using the retrieved values
        keys.insert(0, 'version')
        keys.insert(0, 'test')
        df = pd.DataFrame(file_values, columns=keys)

        # Sort the DataFrame based on the values of the first key (i.e., "name")
        df = df.sort_values(by=keys[0])

        # Create a new DataFrame to store the results
        result_df = pd.DataFrame(columns=keys)

        # Initialize the last key value
        last_key_value = None

        # Iterate over the rows in the DataFrame
        for index, row in df.iterrows():

            # Check if the value of the first key has changed
            if row[keys[0]] != last_key_value:
                # If yes, create a new row in the result DataFrame with the minimum values of the previous rows
                result_row = df.loc[df[keys[0]] == last_key_value].min()
                result_df = result_df.append(result_row, ignore_index=True)

                # Update the last key value
                last_key_value = row[keys[0]]

            # Update the last key value if it is None
            if last_key_value is None:
                last_key_value = row[keys[0]]

        # Add the last row to the result DataFrame
        result_row = df.loc[df[keys[0]] == last_key_value].min()
        result_df = result_df.append(result_row, ignore_index=True)

        # Save the DataFrame as a CSV file
        result_df.to_csv(directory+"output_2.csv", index=False)

        # Print confirmation message
        print("CSV file saved successfully!")

        # Print the DataFrame
        print(result_df)

    def fix_malformed_json(self, file_path, output_file_path):
        with open(file_path, 'r') as file:
            content = file.read()

        # Replace '}\n{' with '},\n{'
        content = content.replace('}\n{', '},\n{')

        # Add enclosing brackets to make it a valid JSON array
        content = f"[{content}]"

        # Write fixed JSON to a new file
        with open(output_file_path, 'w') as output_file:
            output_file.write(content)

        # Load the fixed JSON and return it
        with open(output_file_path, 'r') as fixed_file:
            fixed_json = json.load(fixed_file)

        return fixed_json

    def delete_files_in_directory(self, directory):
        for root, dirs, files in os.walk(directory):
            for file in files:
                file_path = os.path.join(root, file)
                os.remove(file_path)
                print(f"File {file_path} deleted.")

    def delete_empty_subdirectories(self, directory):
        for root, dirs, files in os.walk(directory, topdown=False):
            for dir in dirs:
                dir_path = os.path.join(root, dir)
                if not os.listdir(dir_path):
                    os.rmdir(dir_path)
                    print("Deleted empty directory:", dir_path)

# ------------------------------------------------------------------------

jr = JsonResults()

# Directory containing JSON files
directory = "/home/morteza/Documents/Morteza/CurrentAPF/result_apf/"
# List of keys to retrieve values for
keys = ["robot_count", "mean_len", "total_len", "operation_time", "total_time"]
jr.create_table(directory, keys)

# dctory containing JSON files
directory = "/home/morteza/Documents/Morteza/CurrentAPF/result_apf/"
# List of keys to retrieve values for
keys = ["robot_count", "mean_len", "total_len", "operation_time", "total_time"]
jr.create_table_2(directory, keys)


# #
# directory = "/home/morteza/Documents/Morteza/CurrentAPF"
# jr.delete_files_in_directory(directory)


# # directory = "/home/morteza/Documents/Morteza/CurrentAPF"
# directory = '/home/morteza/Documents/Morteza/Results-APF'
# jr.delete_empty_subdirectories(directory)

