# Physical AI & Humanoid Robotics Textbook

An AI-native textbook covering Physical AI and Humanoid Robotics, built with Docusaurus and integrated with a RAG-powered chatbot for interactive learning.

## Overview

This comprehensive textbook guides students and robotics developers through:

- **Module 1**: The Robotic Nervous System (ROS 2)
- **Module 2**: Simulation Environments (Gazebo & Unity)
- **Module 3**: NVIDIA Isaac Sim
- **Module 4**: Vision-Language-Action Models
- **Module 5**: Capstone Project

## Features

- 📚 12+ chapters of in-depth robotics content
- 💻 200+ runnable code examples (Python, URDF, Bash)
- 📊 Interactive diagrams using Mermaid
- 🤖 RAG-powered chatbot for answering questions from the textbook
- ✅ Validated code examples (tested in ROS 2 Humble)
- 🌐 Deployed on GitHub Pages

## Prerequisites

- **Node.js** 18+ (for Docusaurus)
- **Python** 3.10+ (for code examples and validation scripts)
- **ROS 2 Humble** (recommended for following along with examples)
- **Git** (for version control)

## Installation

### Local Development

1. Clone the repository:
   ```bash
   git clone https://github.com/your-org/books.git
   cd books
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm run start
   ```

4. Open [http://localhost:3000](http://localhost:3000) in your browser

### Building for Production

```bash
npm run build
```

The static files will be generated in the `build/` directory.

## Project Structure

```
books/
├── docs/                   # Markdown content
│   ├── intro.md
│   ├── module-1-ros2/
│   ├── module-2-simulation/
│   └── ...
├── src/                    # React components
│   ├── components/
│   └── css/
├── static/                 # Static assets
│   ├── img/
│   ├── code/              # Downloadable code examples
│   └── files/
├── scripts/               # Validation scripts
├── docusaurus.config.js   # Docusaurus configuration
├── sidebars.js            # Sidebar navigation
└── package.json
```

## Available Commands

```bash
npm run start              # Start development server
npm run build              # Build for production
npm run serve              # Serve built site locally
npm run validate-links     # Check for broken links
npm run lint               # Lint markdown files
```

## Validation Scripts

Code examples are validated using automated scripts:

```bash
# Validate Python code syntax
python scripts/validate_examples.py

# Validate URDF files
python scripts/validate_urdf.py

# Run all validations
python scripts/validate_all.py
```

## Contributing

Contributions are welcome! Please see our [Contributing Guide](CONTRIBUTING.md) for details.

### Adding a New Chapter

1. Create a new markdown file in the appropriate module directory
2. Add YAML frontmatter with metadata
3. Update `sidebars.js` to include the new chapter
4. Test locally with `npm run start`

See [specs/001-ros2-basics/quickstart.md](specs/001-ros2-basics/quickstart.md) for detailed instructions.

## RAG Chatbot

The textbook includes an integrated RAG (Retrieval-Augmented Generation) chatbot that answers questions based solely on the book content.

**Features:**
- Answers questions using OpenAI embeddings + Qdrant vector DB
- Chapter-level filtering (query specific modules)
- Zero hallucination tolerance (answers only from book content)
- Conversation history tracking with Neon Postgres

See [rag-backend/README.md](rag-backend/README.md) for setup instructions.

## Technology Stack

- **Frontend**: Docusaurus 3.x, React 18+, MDX
- **Diagrams**: Mermaid
- **Backend (RAG)**: FastAPI, OpenAI API, Qdrant Cloud, Neon Postgres
- **Validation**: Python (AST parsing, Ruff, check_urdf)
- **Deployment**: GitHub Pages, GitHub Actions
- **Testing**: Docker (ROS 2 Humble)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS 2 community for excellent documentation
- NVIDIA for Isaac Sim resources
- OpenAI for GPT and embedding models
- Docusaurus team for the amazing framework

## Contact

For questions or support, please open an issue on GitHub.

---

**Built with ❤️ by the Physical AI Team**
