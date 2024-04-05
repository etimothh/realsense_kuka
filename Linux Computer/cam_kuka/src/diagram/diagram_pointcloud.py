from graphviz import Digraph

# Create a Digraph object
dot = Digraph()

# Add nodes
dot.node('RealPointCloud (Python)', 'RealPointCloud (Python)',)
dot.node('Sender', 'Sender (Python)')
dot.node('Receive_pcd (Java)', 'Receive_pcd (Java)')
dot.node('Treatment PCD (Java)', 'Treatment PCD (Java)')

# Add edges
dot.edge('RealPointCloud (Python)', 'Sender')
dot.edge('Sender', 'Receive_pcd (Java)')
dot.edge('Receive_pcd (Java)', 'Sender')
dot.edge('Receive_pcd (Java)', 'Treatment PCD (Java)')



# Render and save the diagram
dot.render('diagram_pcl', format='png', cleanup=True)

print("Diagram generated and saved as diagram.png")
