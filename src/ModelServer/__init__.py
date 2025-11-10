from .web.client import ModelClient
from .web.server import start_server
from .config.model_server_config import MODEL_SERVER_URLS

hexmove_local = ModelClient(MODEL_SERVER_URLS['hexmove_local'])
hexmove_remote = ModelClient(MODEL_SERVER_URLS['hexmove_remote'])
hexmove = hexmove_local

utils_local = ModelClient(MODEL_SERVER_URLS['utils_local'])
utils = utils_local

go2_foxy_local = ModelClient(MODEL_SERVER_URLS['go2_foxy_local'])
go2_foxy_remote = ModelClient(MODEL_SERVER_URLS['go2_foxy_remote'])
go2_foxy = go2_foxy_local

go2_noetic_local = ModelClient(MODEL_SERVER_URLS['go2_noetic_local'])
go2_noetic_remote = ModelClient(MODEL_SERVER_URLS['go2_noetic_remote'])
go2_noetic = go2_noetic_local
