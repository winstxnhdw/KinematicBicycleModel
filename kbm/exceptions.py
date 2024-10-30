class NegativeValueError(ValueError):
    def __init__(self, argument: str) -> None:
        super().__init__(f"{argument} must be positive.")
