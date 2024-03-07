import os
import xml.etree.ElementTree as ET
import argparse

def combine_xml_files(input_dir, output_dir):
    total_time = 0.0
    combined_root = ET.Element("combined_testsuite")
    
    for filename in os.listdir(input_dir):
        if filename.endswith(".xml"):
            file_path = os.path.join(input_dir, filename)
            print(file_path)
            
            tree = ET.parse(file_path)
            root = tree.getroot()
            time = float(root.attrib.get("time", 0))
            total_time += time
            combined_root.append(root)
    
    # Update the time attribute of the combined root element
    combined_root.set("time", str(total_time))
    
    # Convert combined_root to a string and replace '><' with '>\n<'
    combined_str = ET.tostring(combined_root, encoding="utf-8", method="xml").decode("utf-8").replace('><', '>\n<')
    
    # Write the combined XML to a file with proper indentation
    with open(output_dir, "wb") as combined_file:
        combined_file.write(combined_str.encode("utf-8"))
    
    print(f"Combined XML written to {output_dir}")

def main():
    parser = argparse.ArgumentParser(description="Combine XML files in a directory and calculate total time.")
    parser.add_argument("-i", "--input_dir", help="Path to the directory containing XML files.")
    parser.add_argument("-o", "--output_dir", help="Path to the combined XML file.")

    args = parser.parse_args()
    combine_xml_files(args.input_dir, args.output_dir)

if __name__ == "__main__":
    main()
