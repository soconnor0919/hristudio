"use client";
"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.api = void 0;
exports.TRPCReactProvider = TRPCReactProvider;
var react_query_1 = require("@tanstack/react-query");
var client_1 = require("@trpc/client");
var react_query_2 = require("@trpc/react-query");
var react_1 = require("react");
var superjson_1 = require("superjson");
var query_client_1 = require("./query-client");
var clientQueryClientSingleton = undefined;
var getQueryClient = function () {
    if (typeof window === "undefined") {
        // Server: always make a new query client
        return (0, query_client_1.createQueryClient)();
    }
    // Browser: use singleton pattern to keep the same query client
    clientQueryClientSingleton !== null && clientQueryClientSingleton !== void 0 ? clientQueryClientSingleton : (clientQueryClientSingleton = (0, query_client_1.createQueryClient)());
    return clientQueryClientSingleton;
};
exports.api = (0, react_query_2.createTRPCReact)();
function TRPCReactProvider(props) {
    var queryClient = getQueryClient();
    var trpcClient = (0, react_1.useState)(function () {
        return exports.api.createClient({
            links: [
                (0, client_1.loggerLink)({
                    enabled: function (op) {
                        return process.env.NODE_ENV === "development" ||
                            (op.direction === "down" && op.result instanceof Error);
                    },
                }),
                (0, client_1.httpBatchStreamLink)({
                    transformer: superjson_1.default,
                    url: getBaseUrl() + "/api/trpc",
                    headers: function () {
                        var headers = new Headers();
                        headers.set("x-trpc-source", "nextjs-react");
                        return headers;
                    },
                }),
            ],
        });
    })[0];
    return (<react_query_1.QueryClientProvider client={queryClient}>
      <exports.api.Provider client={trpcClient} queryClient={queryClient}>
        {props.children}
      </exports.api.Provider>
    </react_query_1.QueryClientProvider>);
}
function getBaseUrl() {
    var _a;
    if (typeof window !== "undefined")
        return window.location.origin;
    if (process.env.VERCEL_URL)
        return "https://".concat(process.env.VERCEL_URL);
    return "http://localhost:".concat((_a = process.env.PORT) !== null && _a !== void 0 ? _a : 3000);
}
