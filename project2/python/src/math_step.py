import math

FUNCTIONS = {"sin", "cos", "tan", "exp", "sqrt", "ln", "abs"}


class ExprError(Exception):
    pass


def tokenize(expression):
    tokens = []
    i = 0

    while i < len(expression):
        char = expression[i]

        if char.isspace():
            i += 1
            continue

        if char in "+-*/^()":
            tokens.append((char, char))
            i += 1
            continue

        if char.isdigit() or char == ".":
            start = i
            has_dot = False

            while i < len(expression):
                current = expression[i]

                if current.isdigit():
                    i += 1
                elif current == "." and not has_dot:
                    has_dot = True
                    i += 1
                else:
                    break

            number_text = expression[start:i]

            if number_text == ".":
                raise ExprError("malformed number")

            try:
                number = float(number_text)
            except ValueError:
                raise ExprError("malformed number")

            tokens.append(("NUM", number))
            continue

   
        if char.isalpha() or char == "_":
            start = i

            while i < len(expression):
                current = expression[i]

                if current.isalnum() or current == "_":
                    i += 1
                else:
                    break

            name = expression[start:i]
            tokens.append(("IDENT", name))
            continue

        raise ExprError(f"unrecognized character '{char}'")

    return tokens


class Parser:
    def __init__(self, tokens):
        self.tokens = tokens
        self.position = 0

    def peek(self):
        if self.position < len(self.tokens):
            return self.tokens[self.position]
        return None

    def advance(self):
        token = self.peek()
        self.position += 1
        return token

    def parse(self):
        if not self.tokens:
            raise ExprError("empty expression")

        result = self.parse_expression()

       
        if self.position != len(self.tokens):
            raise ExprError("trailing garbage")

        return result

    def parse_expression(self):
        result = self.parse_term()

        while True:
            token = self.peek()

            if token and token[0] in ("+", "-"):
                self.advance()
                right = self.parse_term()
                result = ("bin", token[0], result, right)
            else:
                return result

    def parse_term(self):
        result = self.parse_power()

        while True:
            token = self.peek()

            if token and token[0] in ("*", "/"):
                self.advance()
                right = self.parse_power()
                result = ("bin", token[0], result, right)
            else:
                return result

    def parse_power(self):
        result = self.parse_unary()
        token = self.peek()

        if token and token[0] == "^":
            self.advance()
            right = self.parse_power()
            return ("bin", "^", result, right)

        return result

    def parse_unary(self):
        token = self.peek()

        if token and token[0] == "-":
            self.advance()
            return ("neg", self.parse_unary())

        return self.parse_atom()

    def parse_atom(self):
        token = self.peek()

        if token is None:
            raise ExprError("unexpected end of expression")

        kind, value = token

        if kind == "NUM":
            self.advance()
            return ("num", value)


        if kind == "(":
            self.advance()

            result = self.parse_expression()

            closing = self.peek()

            if not closing or closing[0] != ")":
                raise ExprError("unbalanced parentheses")

            self.advance()
            return result

        if kind == "IDENT":
            self.advance()
           
            if value == "t":
                return ("var",)

            if value in FUNCTIONS:
                opening = self.peek()

                if not opening or opening[0] != "(":
                    raise ExprError(f"expected '(' after function '{value}'")

                self.advance()

                argument = self.parse_expression()

                closing = self.peek()

                if not closing or closing[0] != ")":
                    raise ExprError("unbalanced parentheses")

                self.advance()

                return ("call", value, argument)

            raise ExprError(f"unknown identifier '{value}'")

        raise ExprError(f"unexpected token '{value}'")


def parse_expression(expression):
    if expression is None or expression == "":
        raise ExprError("empty expression")

    tokens = tokenize(expression)
    parser = Parser(tokens)

    return parser.parse()


def safe_sqrt(value):
    if value < 0:
        return float("nan")

    return math.sqrt(value)


def safe_ln(value):
    if value < 0:
        return float("nan")

    if value == 0:
        return float("-inf")

    return math.log(value)


def safe_exp(value):
    try:
        return math.exp(value)
    except OverflowError:
        return float("inf")


def safe_power(left, right):
    try:
        result = left ** right

        if isinstance(result, complex):
            return float("nan")

        return float(result)

    except OverflowError:
        if left < 0:
            return float("nan")

        return float("inf")

    except ZeroDivisionError:
        return float("inf")


FUNCTION_CALLS = {
    "sin": math.sin,
    "cos": math.cos,
    "tan": math.tan,
    "exp": safe_exp,
    "sqrt": safe_sqrt,
    "ln": safe_ln,
    "abs": abs
}


def evaluate(node, t):
    node_type = node[0]

 
    if node_type == "num":
        return node[1]

    if node_type == "var":
        return t

    if node_type == "neg":
        return -evaluate(node[1], t)

    if node_type == "call":
        function_name = node[1]
        argument = node[2]

        try:
            value = evaluate(argument, t)
            return float(FUNCTION_CALLS[function_name](value))
        except ValueError:
            return float("nan")

    if node_type == "bin":
        operator = node[1]

        left = evaluate(node[2], t)
        right = evaluate(node[3], t)

        if operator == "+":
            return left + right

        if operator == "-":
            return left - right

        if operator == "*":
            return left * right

        if operator == "/":
            if right == 0:
                if left == 0:
                    return float("nan")

                if left > 0:
                    return float("inf")

                return float("-inf")

            return left / right

        if operator == "^":
            return safe_power(left, right)

    raise ExprError("unknown expression node")


def make_forcing_function(expression):
    # Parse the expression once instead of parsing it every time
    tree = parse_expression(expression)

    # Return a function that only needs t
    return lambda t: evaluate(tree, t)