from graphviz import Digraph

# Create Digraph object
dot = Digraph()

# Add nodes
dot.node('image_sender (Python)', 'image_sender (Python)')
dot.node('receive_image (Java)', 'receive_image (Java)')

# Add edges
dot.edge('image_sender (Python)', 'receive_image (Java)', label='Send image data')
dot.edge('receive_image (Java)', 'image_sender (Python)', label='Send trigger command')

# Save diagram
dot.render('image_receiver_trigger_server_diagram', format='png', cleanup=True)
