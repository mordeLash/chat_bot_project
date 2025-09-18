import pandas as pd
from typing import List, Any
import re

def sanitize_filename(filename: str) -> str:
    """
    Sanitize a filename so it can be safely used on all operating systems.

    Args:
        filename (str): Original filename.

    Returns:
        str: Sanitized filename safe for saving.
    """
    # Replace invalid characters with underscores
    sanitized = re.sub(r'[<>:"\\|?*]', "_", filename)
    # Strip whitespace and remove invalid trailing characters
    sanitized = sanitized.strip().rstrip(". ")
    return sanitized

        
def save_objects_to_excel(object_list: List[Any], output_file: str) -> None:
    """
    Save a list of Python objects or dictionaries into an Excel (.xlsx) file.

    Args:
        object_list (List[Any]): List of objects or dictionaries to save.
            - If an element is a dict, it is used directly.
            - If it is an object, its attributes (non-private) are extracted via vars().
        output_file (str): Target Excel filename.

    Raises:
        ValueError: If the object_list is empty.
        Exception: If saving to Excel fails.

    """
    # Ensure list is not empty
    if not object_list:
        raise ValueError("Object list cannot be empty")
    
    # Convert objects to dictionaries (ignoring private attributes)
    data = []
    for obj in object_list:
        if isinstance(obj, dict):
            data.append(obj)
        else:
            obj_dict = {
                key: value 
                for key, value in vars(obj).items()
                if not key.startswith('_')  # Skip private/internal fields
            }
            data.append(obj_dict)
    
    # Convert to pandas DataFrame
    df = pd.DataFrame(data)
    
    # Sanitize output filename
    output_file = sanitize_filename(output_file)
    
    # Write DataFrame to Excel
    try:
        df.to_excel(output_file, index=False)
        print(f"Successfully saved data to {output_file}")
    except Exception as e:
        print(f"Error saving to Excel: {str(e)}")
