# import logging
from colorama import Fore, Style


class MyLogger:
    def __init__(self, name="mh"):
        self.name: str = name

    def info1(self, message: str) -> None:
        print(f"{Fore.CYAN}[{self.name}]: {message}{Style.RESET_ALL}")

    def info2(self, message: str) -> None:
        print(f"{Fore.GREEN}[{self.name}]: {message}{Style.RESET_ALL}")

    def info3(self, message: str) -> None:
        pass
        # print(f"{Fore.WHITE}[{self.name}]: {message}{Style.RESET_ALL}")

    def info(self, message: str) -> None:
        print(f"{Fore.WHITE}[{self.name}]: {message}{Style.RESET_ALL}")

    def debug(self, message: str) -> None:
        print(f"{Fore.BLUE}[{self.name}]: {message}{Style.RESET_ALL}")

    def warn(self, message: str) -> None:
        print(f"{Fore.YELLOW}[{self.name}]: {message}{Style.RESET_ALL}")

    def error(self, message: str) -> None:
        print(f"{Fore.RED}[{self.name}]: {message}{Style.RESET_ALL}")

# class ColorFormatter(logging.Formatter):
#     COLORS = {
#         logging.DEBUG: Fore.CYAN,
#         logging.INFO: Fore.GREEN,
#         logging.WARNING: Fore.YELLOW,
#         logging.ERROR: Fore.RED,
#         logging.CRITICAL: Fore.MAGENTA,
#     }

#     def format(self, record):
#         color = self.COLORS.get(record.levelno, Fore.WHITE)
#         msg = super().format(record)
#         return f"{color}{msg}{Style.RESET_ALL}"


# class MyLogger1:
#     def __init__(self, name="mh", level=logging.DEBUG):
#         self.logger = logging.getLogger(name)
#         self.logger.setLevel(level)

#         # Create console handler
#         ch = logging.StreamHandler()
#         ch.setLevel(level)

#         # Create formatter and add it to the handler
#         # formatter = ColorFormatter(fmt='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
#         formatter = ColorFormatter(fmt='%(name)s: %(message)s')
#         ch.setFormatter(formatter)

#         # Add the handler to the logger
#         self.logger.addHandler(ch)

#     def debug(self, message):
#         self.logger.debug(message)

#     def info(self, message):
#         self.logger.info(message)

#     def warning(self, message):
#         self.logger.warning(message)

#     def error(self, message):
#         self.logger.error(message)


if __name__ == "__main__":
    mg = MyLogger()
    mg.info1("This is an info message with cyan color")
    mg.info2("This is an info message with green color")
    mg.info("This is an info message with yellow color")
    mg.warn("This is an info message with yellow color")
    mg.error("This is an info message with yellow color")

    # mlogger = MyLogger1()
    # mlogger.debug("debug message")
    # mlogger.info("Info message")
    # mlogger.warning("Warning message")
    # mlogger.error("Error message")
