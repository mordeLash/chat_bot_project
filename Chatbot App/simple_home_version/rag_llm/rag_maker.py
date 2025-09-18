"""simple app to build a VDB on some folders"""
from rag_database import RAGDatabase
def main():
    db = RAGDatabase()
    db.add_documents_from_folder("./rag_data/signalProcessing")
    # db.add_documents_from_folder("./rag_data/ROS")
    # db.add_documents_from_folder("./rag_data/volleyball")






if __name__ == "__main__":
    main()