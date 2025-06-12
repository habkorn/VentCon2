import gzip
import os
import shutil

def compress_file(input_file, output_file):
    """Compress a file using gzip"""
    try:
        with open(input_file, 'rb') as f_in:
            with gzip.open(output_file, 'wb', compresslevel=9) as f_out:
                shutil.copyfileobj(f_in, f_out)
        
        # Get file sizes for comparison
        original_size = os.path.getsize(input_file)
        compressed_size = os.path.getsize(output_file)
        compression_ratio = (1 - compressed_size / original_size) * 100
        
        print(f"Compressed {input_file}")
        print(f"  Original size: {original_size:,} bytes")
        print(f"  Compressed size: {compressed_size:,} bytes") 
        print(f"  Compression: {compression_ratio:.1f}%")
        print()
        
    except FileNotFoundError:
        print(f"Warning: {input_file} not found, skipping...")
    except Exception as e:
        print(f"Error compressing {input_file}: {e}")

def main():
    """Compress all web resources for ESP32"""
    data_dir = "data"
    
    # Ensure data directory exists
    os.makedirs(data_dir, exist_ok=True)
    
    # Files to compress (add your actual files here)
    files_to_compress = [
        ("chart.min.js", "data/chart.min.js.gz"),
        ("moment.min.js", "data/moment.min.js.gz"), 
        ("chartjs-adapter-moment.min.js", "data/chartjs-adapter-moment.min.js.gz"),
        ("Logo.svg", "data/Logo.svg.gz")
    ]
    
    print("Compressing web resources for ESP32...")
    print("=" * 50)
    
    for input_file, output_file in files_to_compress:
        compress_file(input_file, output_file)
    
    print("Compression complete!")
    print("\nTo use these files:")
    print("1. Upload the .gz files to ESP32 LittleFS using the Arduino IDE data upload tool")
    print("2. The ESP32 web server will automatically serve .gz files when clients accept gzip encoding")
    print("3. This reduces bandwidth usage and improves loading times")

if __name__ == "__main__":
    main()
