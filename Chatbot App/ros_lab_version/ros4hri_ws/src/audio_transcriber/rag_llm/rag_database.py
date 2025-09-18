import json
import faiss
import numpy as np
from pathlib import Path
from datetime import datetime

from langchain_community.document_loaders import PyMuPDFLoader, TextLoader, UnstructuredMarkdownLoader
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_ollama import OllamaEmbeddings
from langchain_community.docstore.in_memory import InMemoryDocstore
from langchain_community.vectorstores import FAISS


class RAGDatabase:
    """
    Retrieval-Augmented Generation (RAG) database using FAISS.
    Handles embeddings, indexing, and querying of documents.
    """

    def __init__(self, persist_dir: Path = None):
        # Default persistence directory: rag_llm/faiss_index
        base_path = Path(__file__).resolve().parent
        self.persist_dir = persist_dir or (base_path / "faiss_index")
        self.persist_dir.mkdir(parents=True, exist_ok=True)

        # Use normalized Ollama embeddings
        self.embeddings = NormalizedOllamaEmbeddings(model="nomic-embed-text")

        index_path = self.persist_dir / "index.faiss"
        if index_path.exists():
            # Load existing FAISS index
            self.vector_store = FAISS.load_local(
                folder_path=self.persist_dir,
                embeddings=self.embeddings,
                allow_dangerous_deserialization=True,
            )
            print(f"LOG: Loaded FAISS index from '{self.persist_dir}'.")
        else:
            # Create new FAISS index
            self.createDB()
            self.vector_store.save_local(self.persist_dir)
            print(f"LOG: Created new FAISS index at '{self.persist_dir}'.")

    def queryDB(self, prompt: str, k: int = 3, n: float = 0.6, log_to_file: bool = True) -> str:
        """
        Query the FAISS index for documents relevant to a prompt.
        Returns concatenated text chunks.
        """
        docs_and_scores = self.vector_store.similarity_search_with_score(prompt, k)
        filtered_docs = [doc for doc, score in docs_and_scores if score > n]
        texts = [doc.page_content for doc in filtered_docs]

        # Log queries if enabled
        if log_to_file:
            log_entry = {
                "timestamp": datetime.now().isoformat(),
                "prompt": prompt,
                "results": [
                    {"content": doc.page_content, "score": float(score)}
                    for doc, score in docs_and_scores
                ],
            }
            log_path = self.persist_dir / "query_log.jsonl"
            with open(log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(log_entry, ensure_ascii=False) + "\n")

        return "\n\n---\n\n".join(texts)

    def createDB(self):
        """Create a new FAISS index with normalized embeddings."""
        embeddings = NormalizedOllamaEmbeddings(model="nomic-embed-text")
        embedding_dim = len(embeddings.embed_query("hello world"))
        index = faiss.IndexFlatIP(embedding_dim)  # inner product index

        self.vector_store = FAISS(
            embedding_function=embeddings,
            index=index,
            docstore=InMemoryDocstore(),
            index_to_docstore_id={},
        )

    def add_documents_from_folder(self, folder_path: Path):
        """
        Add documents from a folder into the FAISS index.
        Supports PDF, TXT, and Markdown.
        """
        folder_path = Path(folder_path)
        files_added = 0
        all_splits = []
        chunk_log_path = self.persist_dir / "chunk_log.txt"

        def preprocess(text: str) -> str:
            """Remove line breaks and extra spaces for cleaner chunks."""
            return " ".join(text.replace("\n", " ").split())

        for filename in folder_path.iterdir():
            if not filename.is_file():
                continue

            ext = filename.suffix.lower()
            try:
                # Pick loader based on file extension
                if ext == ".pdf":
                    loader = PyMuPDFLoader(str(filename))
                elif ext == ".txt":
                    loader = TextLoader(str(filename), encoding="utf-8")
                elif ext == ".md":
                    loader = UnstructuredMarkdownLoader(str(filename))
                else:
                    print(f"LOG: Skipped unsupported file type: {filename.name}")
                    continue

                pages = list(loader.lazy_load())

                # Preprocess text
                for doc in pages:
                    doc.page_content = preprocess(doc.page_content)

                # Split into chunks
                text_splitter = RecursiveCharacterTextSplitter(
                    chunk_size=1000,
                    chunk_overlap=200,
                    add_start_index=True,
                )
                splits = text_splitter.split_documents(pages)

                # Add metadata + save chunks for debugging
                for i, doc in enumerate(splits):
                    doc.metadata["source_file"] = filename.name
                    doc.metadata["chunk_index"] = i
                    with open(chunk_log_path, "a", encoding="utf-8") as f:
                        f.write(
                            f"Chunk {i} from {filename.name}:\n"
                            f"{doc.page_content}\n\n{'-'*60}\n\n"
                        )

                all_splits.extend(splits)
                files_added += 1
                print(f"LOG: Processed '{filename.name}' into {len(splits)} chunks.")

            except Exception as e:
                print(f"ERROR: Failed to process {filename.name}: {e}")

        # Add to FAISS and save
        if all_splits:
            self.vector_store.add_documents(all_splits)
            self.vector_store.save_local(self.persist_dir)
            print(
                f"LOG: Added {len(all_splits)} chunks from {files_added} files "
                f"and updated FAISS index."
            )
        else:
            print("LOG: No documents were added.")


class NormalizedOllamaEmbeddings(OllamaEmbeddings):
    """
    Ollama embeddings wrapper that normalizes vectors to unit length (L2 norm).
    Ensures FAISS inner product search behaves like cosine similarity.
    """

    def __init__(self, model="nomic-embed-text"):
        super().__init__(model=model, base_url="http://132.70.226.188:11435/")

    def embed_documents(self, texts):
        vectors = np.array(super().embed_documents(texts)).astype("float32")
        faiss.normalize_L2(vectors)
        return vectors.tolist()

    def embed_query(self, text):
        vector = np.array([super().embed_query(text)]).astype("float32")
        faiss.normalize_L2(vector)
        return vector[0].tolist()
