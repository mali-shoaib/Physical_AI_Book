"""
Embedding Pipeline for Physical AI Textbook RAG System
Crawls Docusaurus site, chunks text, generates Cohere embeddings, stores in Qdrant
"""

import os
import uuid
import re
from typing import List, Tuple, Dict
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

import requests
from bs4 import BeautifulSoup
import tiktoken
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type
from loguru import logger
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
DOCUSAURUS_BASE_URL = os.getenv("DOCUSAURUS_BASE_URL", "https://physical-ai-robotics-textbook-xi.vercel.app")
CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "768"))
CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "150"))
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

# Initialize logger
logger.remove()
logger.add(
    lambda msg: print(msg, end=""),
    format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <level>{message}</level>",
    level=LOG_LEVEL
)

# Initialize tokenizer
tokenizer = tiktoken.get_encoding("cl100k_base")


def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl Docusaurus sitemap and return all documentation URLs.

    Args:
        base_url: Docusaurus base URL (e.g., "https://example.com")

    Returns:
        List of filtered documentation URLs (only /docs/* paths)
    """
    logger.info(f"Fetching sitemap from {base_url}/sitemap.xml")

    try:
        sitemap_url = f"{base_url}/sitemap.xml"
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()

        # Parse XML sitemap
        root = ET.fromstring(response.content)

        # Extract URLs from sitemap (handle XML namespaces)
        namespace = {'ns': 'http://www.sitemaps.org/schemas/sitemap/0.9'}
        urls = []

        for url_elem in root.findall('.//ns:url/ns:loc', namespace):
            url = url_elem.text
            # Filter: only /docs/* paths, exclude /blog, /api
            if url and '/docs/' in url:
                urls.append(url)

        logger.info(f"Discovered {len(urls)} documentation URLs")
        return urls

    except Exception as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        raise


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type((requests.RequestException, requests.Timeout))
)
def extract_text_from_url(url: str) -> Tuple[str, Dict]:
    """
    Fetch HTML and extract cleaned text with metadata.

    Args:
        url: Full URL of Docusaurus page

    Returns:
        Tuple of (cleaned_text, metadata)
    """
    logger.info(f"Extracting text from {url}")

    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'lxml')

        # Extract main content (Docusaurus uses article.markdown or .theme-doc-markdown)
        content_elem = soup.find('article', class_='markdown') or \
                      soup.find('article', class_='theme-doc-markdown') or \
                      soup.find('main')

        if not content_elem:
            logger.warning(f"No main content found for {url}")
            return "", {}

        # Remove navigation, footer, table of contents
        for tag in content_elem.find_all(['nav', 'footer']):
            tag.decompose()
        for tag in content_elem.find_all(class_=['navbar', 'footer', 'table-of-contents', 'pagination-nav']):
            tag.decompose()

        # Extract text
        text = content_elem.get_text(separator='\n', strip=True)

        # Extract heading hierarchy
        headings = []
        for heading in content_elem.find_all(['h1', 'h2', 'h3', 'h4']):
            headings.append(heading.get_text(strip=True))

        # Derive metadata from URL
        url_path = url.replace(DOCUSAURUS_BASE_URL, '').strip('/')
        parts = url_path.split('/')

        # Extract module and chapter from URL path
        module_name = parts[1] if len(parts) > 1 else "Unknown Module"
        chapter_id = '/'.join(parts[1:]) if len(parts) > 1 else url_path

        metadata = {
            "source_url": url,
            "chapter_id": chapter_id,
            "module_name": module_name.replace('-', ' ').title(),
            "heading_hierarchy": headings,
            "fetch_timestamp": datetime.now(timezone.utc).isoformat()
        }

        logger.info(f"Extracted {len(text)} characters from {url}")
        return text, metadata

    except Exception as e:
        logger.error(f"Failed to extract text from {url}: {e}")
        raise


def chunk_text(text: str, metadata: Dict) -> List[Tuple[str, Dict]]:
    """
    Split text into semantic chunks with overlap.

    Args:
        text: Cleaned text from extract_text_from_url
        metadata: Page metadata to propagate to chunks

    Returns:
        List of (chunk_text, chunk_metadata) tuples
    """
    logger.info(f"Chunking text for {metadata.get('source_url', 'unknown')}")

    if not text or len(text) < 100:
        logger.warning("Text too short, skipping chunking")
        return []

    chunks = []

    # Split on double newlines first (paragraphs)
    paragraphs = re.split(r'\n\n+', text)

    current_chunk = ""
    current_tokens = 0

    for paragraph in paragraphs:
        paragraph = paragraph.strip()
        if not paragraph:
            continue

        # Count tokens
        para_tokens = len(tokenizer.encode(paragraph))

        # If adding this paragraph exceeds chunk size, save current chunk
        if current_tokens + para_tokens > CHUNK_SIZE and current_chunk:
            chunks.append(current_chunk)

            # Add overlap (last part of current chunk)
            overlap_text = ' '.join(current_chunk.split()[-CHUNK_OVERLAP:])
            current_chunk = overlap_text + '\n\n' + paragraph
            current_tokens = len(tokenizer.encode(current_chunk))
        else:
            current_chunk += '\n\n' + paragraph if current_chunk else paragraph
            current_tokens += para_tokens

    # Add final chunk
    if current_chunk:
        chunks.append(current_chunk)

    # Create chunk metadata
    chunk_tuples = []
    for i, chunk in enumerate(chunks):
        chunk_meta = metadata.copy()
        chunk_meta.update({
            "chunk_id": str(uuid.uuid4()),
            "chunk_text": chunk,
            "token_count": len(tokenizer.encode(chunk)),
            "chunk_index": i,
            "total_chunks": len(chunks),
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "model_version": "embed-english-v3.0"
        })
        chunk_tuples.append((chunk, chunk_meta))

    logger.info(f"Created {len(chunk_tuples)} chunks")
    return chunk_tuples


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type(Exception)
)
def embed(texts: List[str]) -> List[List[float]]:
    """
    Generate Cohere embeddings for batch of texts.

    Args:
        texts: List of chunk texts to embed (max 96 per batch)

    Returns:
        List of 1024-dimensional embedding vectors
    """
    logger.info(f"Generating embeddings for {len(texts)} texts")

    if not COHERE_API_KEY:
        raise ValueError("COHERE_API_KEY not set in environment")

    co = cohere.Client(COHERE_API_KEY)

    try:
        # Cohere API supports up to 96 texts per request
        batch_size = 96
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            response = co.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"  # For ingestion
            )

            # Extract embeddings - response.embeddings is a list of EmbedFloatsResponse objects
            for emb in response.embeddings:
                all_embeddings.append(emb)
            logger.info(f"Embedded batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

        return all_embeddings

    except Exception as e:
        logger.error(f"Failed to generate embeddings: {e}")
        raise


def create_collection(client: QdrantClient, name: str = "rag_embedding"):
    """
    Initialize Qdrant collection with configuration.

    Args:
        client: Initialized Qdrant client instance
        name: Collection name (default: "rag_embedding")
    """
    logger.info(f"Creating Qdrant collection: {name}")

    try:
        # Check if collection exists
        collections = client.get_collections().collections
        collection_names = [c.name for c in collections]

        if name in collection_names:
            logger.info(f"Collection '{name}' already exists, skipping creation")
            return

        # Create collection
        client.create_collection(
            collection_name=name,
            vectors_config=VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimension
                distance=Distance.COSINE
            )
        )

        logger.info(f"Collection '{name}' created successfully")

    except Exception as e:
        logger.error(f"Failed to create collection: {e}")
        raise


@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    retry=retry_if_exception_type(Exception)
)
def save_chunk_to_qdrant(
    client: QdrantClient,
    chunk_text: str,
    embedding: List[float],
    metadata: Dict
):
    """
    Upsert single chunk with embedding and metadata to Qdrant.

    Args:
        client: Initialized Qdrant client instance
        chunk_text: Original chunk text content
        embedding: 1024-dimensional embedding vector
        metadata: Chunk metadata payload
    """
    try:
        point = PointStruct(
            id=metadata["chunk_id"],
            vector=embedding,
            payload=metadata
        )

        client.upsert(
            collection_name="rag_embedding",
            points=[point]
        )

    except Exception as e:
        logger.error(f"Failed to save chunk {metadata.get('chunk_id')}: {e}")
        raise


def main():
    """
    Orchestrate full pipeline execution: crawl → extract → chunk → embed → store
    """
    logger.info("=" * 80)
    logger.info("Starting Embedding Pipeline")
    logger.info("=" * 80)

    # Validate environment variables
    if not COHERE_API_KEY:
        logger.error("COHERE_API_KEY not set in .env file")
        return
    if not QDRANT_URL or not QDRANT_API_KEY:
        logger.error("QDRANT_URL or QDRANT_API_KEY not set in .env file")
        return

    # Initialize Qdrant client
    logger.info(f"Connecting to Qdrant at {QDRANT_URL}")
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )

    # Create collection
    create_collection(qdrant_client)

    # Step 1: Crawl URLs
    urls = get_all_urls(DOCUSAURUS_BASE_URL)
    logger.info(f"Found {len(urls)} URLs to process")

    # Statistics
    stats = {
        "pages_discovered": len(urls),
        "pages_fetched_success": 0,
        "pages_fetched_failed": 0,
        "chunks_created": 0,
        "embeddings_generated": 0,
        "qdrant_upserts_success": 0,
        "qdrant_upserts_failed": 0
    }

    # Process each URL
    for idx, url in enumerate(urls, 1):
        logger.info(f"\n[{idx}/{len(urls)}] Processing: {url}")

        try:
            # Step 2: Extract text
            text, metadata = extract_text_from_url(url)

            if not text:
                logger.warning(f"Skipping {url} - no text extracted")
                stats["pages_fetched_failed"] += 1
                continue

            stats["pages_fetched_success"] += 1

            # Step 3: Chunk text
            chunks = chunk_text(text, metadata)
            stats["chunks_created"] += len(chunks)

            if not chunks:
                logger.warning(f"No chunks created for {url}")
                continue

            # Step 4: Generate embeddings (batch all chunks from this page)
            chunk_texts = [chunk[0] for chunk in chunks]
            embeddings = embed(chunk_texts)
            stats["embeddings_generated"] += len(embeddings)

            # Step 5: Store in Qdrant
            for (text_chunk, chunk_meta), embedding in zip(chunks, embeddings):
                try:
                    save_chunk_to_qdrant(qdrant_client, text_chunk, embedding, chunk_meta)
                    stats["qdrant_upserts_success"] += 1
                except Exception as e:
                    logger.error(f"Failed to save chunk: {e}")
                    stats["qdrant_upserts_failed"] += 1

            logger.info(f"[OK] Completed {url} - {len(chunks)} chunks stored")

        except Exception as e:
            logger.error(f"[FAIL] Failed to process {url}: {e}")
            stats["pages_fetched_failed"] += 1

    # Final summary
    logger.info("\n" + "=" * 80)
    logger.info("Pipeline Execution Summary")
    logger.info("=" * 80)
    logger.info(f"Pages discovered:        {stats['pages_discovered']}")
    logger.info(f"Pages fetched (success): {stats['pages_fetched_success']}")
    logger.info(f"Pages fetched (failed):  {stats['pages_fetched_failed']}")
    logger.info(f"Chunks created:          {stats['chunks_created']}")
    logger.info(f"Embeddings generated:    {stats['embeddings_generated']}")
    logger.info(f"Qdrant upserts (success): {stats['qdrant_upserts_success']}")
    logger.info(f"Qdrant upserts (failed):  {stats['qdrant_upserts_failed']}")

    success_rate = (stats['qdrant_upserts_success'] / stats['chunks_created'] * 100) if stats['chunks_created'] > 0 else 0
    logger.info(f"Success rate:            {success_rate:.1f}%")
    logger.info("=" * 80)
    logger.info("[SUCCESS] Pipeline completed successfully!")


if __name__ == "__main__":
    main()
