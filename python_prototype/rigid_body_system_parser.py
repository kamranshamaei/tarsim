from inc import *


def print_tree(current_node, childattr='children', nameattr='name', indent='', last='updown'):
    if hasattr(current_node, nameattr):
        name = lambda node: getattr(node, nameattr)
    else:
        name = lambda node: str(node)

    children = lambda node: getattr(node, childattr)
    nb_children = lambda node: sum(nb_children(child) for child in children(node)) + 1
    size_branch = {child: nb_children(child) for child in children(current_node)}

    """ Creation of balanced lists for "up" branch and "down" branch. """
    up = sorted(children(current_node), key=lambda node: nb_children(node))
    down = []
    while up and sum(size_branch[node] for node in down) < sum(size_branch[node] for node in up):
        down.append(up.pop())

    """ Printing of "up" branch. """
    for child in up:
        next_last = 'up' if up.index(child) is 0 else ''
        next_indent = '{0}{1}{2}'.format(indent, ' ' if 'up' in last else '│', ' ' * len(name(current_node)))
        print_tree(child, childattr, nameattr, next_indent, next_last)

    """ Printing of current node. """
    if last == 'up': start_shape = '┌'
    elif last == 'down': start_shape = '└'
    elif last == 'updown': start_shape = ' '
    else: start_shape = '├'

    if up: end_shape = '┤'
    elif down: end_shape = '┐'
    else: end_shape = ''

    print('{0}{1}{2}{3}'.format(indent, start_shape, name(current_node), end_shape))

    """ Printing of "down" branch. """
    for child in down:
        next_last = 'down' if down.index(child) is len(down) - 1 else ''
        next_indent = '{0}{1}{2}'.format(indent, ' ' if 'down' in last else '│', ' ' * len(name(current_node)))
        print_tree(child, childattr, nameattr, next_indent, next_last)


def is_in_tree(index, node):
    if node.rigid_body.index is index:
        return True

    for i in range(len(node.children)):
        return is_in_tree(index, node.children[i])

    return False


class RigidBodySystemParser:
    def __init__(self, rigid_body_system=RigidBodySystem()):
        self.rigid_body_system = rigid_body_system
        self.root_rigid_body_index = -1
        self.tree = Node()
        self.joint_index_dict = dict()
        self.joint_value_dict = dict()

        if not self.parse():
            raise Exception('Rigid Body System Error!', 'Parser')

    def parse(self):
        if not self.verify_rigid_body_system(self.rigid_body_system):
            log("Failed to verify rigid body system")
            return False

        if not self.create_rbs_tree(self.rigid_body_system):
            log("Failed to create rigid body system tree")
            return False

        print_tree(self.tree)
        return True

    def verify_rigid_body_system(self, rigid_body_system):
        # Find the tree root (fixed body)
        counter = 0
        self.root_rigid_body_index = -1
        for i in range(len(rigid_body_system.rigid_bodies)):
            if rigid_body_system.rigid_bodies[i].is_fixed:
                counter += 1
                self.root_rigid_body_index = i

        if counter is not 1:
            self.root_rigid_body_index = -1
            log("Number of fixed rigid bodies is not 1")
            return False

        for i in range(len(rigid_body_system.mates)):
            if rigid_body_system.mates[i].bearing_rigid_body_index is rigid_body_system.mates[i].shaft_rigid_body_index:
                log("Rigid body cannot be mated to itself")
                return False

        return True

    def create_rbs_tree(self, rigid_body_system):
        self.tree = Node(rigid_body=rigid_body_system.rigid_bodies[self.root_rigid_body_index])
        if not self.create_children(root=self.tree, current_node=self.tree, rigid_body_system=rigid_body_system):
            log("Failed to create children for root")
            return False
        return True

    def create_children(self, root=Node(), current_node=Node(), rigid_body_system=RigidBodySystem()):
        for i in range(len(rigid_body_system.mates)):
            if not rigid_body_system.mates[i].is_in_tree:
                child_index = None
                if current_node.rigid_body.index is rigid_body_system.mates[i].bearing_rigid_body_index:
                    child_index = rigid_body_system.mates[i].shaft_rigid_body_index
                elif current_node.rigid_body.index is rigid_body_system.mates[i].shaft_rigid_body_index:
                    child_index = rigid_body_system.mates[i].bearing_rigid_body_index

                if child_index is not None:
                    if is_in_tree(child_index, root):
                        log("Joint %d causes a closed chain" % int(child_index))
                        return False
                    else:
                        new_node = Node(
                            parent=current_node,
                            rigid_body=rigid_body_system.rigid_bodies[child_index],
                            mate_to_parent=rigid_body_system.mates[i])
                        rigid_body_system.mates[i].is_in_tree = True

                        self.joint_index_dict[new_node.mate_to_parent.index] = new_node
                        self.joint_value_dict[new_node.mate_to_parent.index] = new_node.mate_to_parent.value

                        if not self.create_children(root, new_node, rigid_body_system):
                            log("Failed to create children for node %s" % new_node.rigid_body.name)
                            return False
        return True

    def get_tree(self):
        return self.tree

    def get_joint_index_dict(self):
        return self.joint_index_dict

    def get_joint_value_dict(self):
        return self.joint_value_dict